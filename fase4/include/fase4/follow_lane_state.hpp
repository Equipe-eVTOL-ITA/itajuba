#include <Eigen/Eigen>
#include <chrono>
#include <algorithm>
#include <cmath>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "vision_fase4.hpp"
#include "fase4/PidController.hpp"
#include "fase4/transformations.hpp"

class FollowLaneState : public fsm::State {
private:
    std::shared_ptr<Drone> drone;
    std::shared_ptr<VisionNode> vision;
    
    // PID Controllers
    std::unique_ptr<PidController> lateral_pid;  // Para alinhamento X
    std::unique_ptr<PidController> angular_pid;  // Para alinhamento do theta (yaw)
    std::unique_ptr<PidController> circle_x_pid; // Para alinhamento X com círculo
    std::unique_ptr<PidController> circle_y_pid; // Para alinhamento Y com círculo
    
    // Controle de estado anterior
    bool previous_circle_detected = false;
    
    // Parâmetros
    float max_velocity;
    float max_yaw_rate;
    float position_tolerance;
    float height;
    float max_angle_for_translation;
    
    // Setpoints para controle (coordenadas normalizadas)
    static constexpr float TARGET_X_NORMALIZED = 0.0f;  // Centro da imagem (0 = centro)
    static constexpr float TARGET_Y_NORMALIZED = 0.0f;  // Centro da imagem (0 = centro)
    static constexpr float TARGET_THETA = 0.0f;         // Theta alvo (vertical) 
   
public:
    FollowLaneState() : fsm::State() {}

    void on_enter(fsm::Blackboard& bb) override {
        auto drone_ptr = bb.get<std::shared_ptr<Drone>>("drone");
        auto vision_ptr = bb.get<std::shared_ptr<VisionNode>>("vision");
        
        if (drone_ptr == nullptr || *drone_ptr == nullptr) {
            this->drone->log("ERROR: Drone pointer is null in FollowLaneState");
            return;
        }
        if (vision_ptr == nullptr || *vision_ptr == nullptr) {
            this->drone->log("ERROR: Vision pointer is null in FollowLaneState");
            return;
        }
        
        this->drone = *drone_ptr;
        this->vision = *vision_ptr;
        
        this->drone->log("STATE: FOLLOW LANE STATE");

        // Obter parâmetros do blackboard
        this->max_velocity = *bb.get<float>("max_horizontal_velocity");
        this->max_yaw_rate = *bb.get<float>("max_yaw_rate");
        this->position_tolerance = *bb.get<float>("position_tolerance");
        this->height = *bb.get<float>("takeoff_height");
        this->max_angle_for_translation = *bb.get<float>("max_angle_for_translation");

        // Inicializar PIDs
        // PID lateral: setpoint = 0 (centro da imagem normalizado)
        float kp_lateral = *bb.get<float>("pid_lateral_kp");
        float ki_lateral = *bb.get<float>("pid_lateral_ki"); 
        float kd_lateral = *bb.get<float>("pid_lateral_kd");
        
        // PID angular: setpoint = 0 (theta vertical)
        float kp_angular = *bb.get<float>("pid_angular_kp");
        float ki_angular = *bb.get<float>("pid_angular_ki");
        float kd_angular = *bb.get<float>("pid_angular_kd");
        
        // PID posicional para círculo: setpoint = 0 (centro da imagem)
        float kp_pos = *bb.get<float>("pid_pos_kp");
        float ki_pos = *bb.get<float>("pid_pos_ki");
        float kd_pos = *bb.get<float>("pid_pos_kd");
        
        this->lateral_pid = std::make_unique<PidController>(
            kp_lateral, ki_lateral, kd_lateral, 
            TARGET_X_NORMALIZED,
            0.02f  // Sample time menor
        );
        
        this->angular_pid = std::make_unique<PidController>(
            kp_angular, ki_angular, kd_angular, 
            TARGET_THETA,
            0.02f  // Sample time menor
        );
        
        this->circle_x_pid = std::make_unique<PidController>(
            kp_pos, ki_pos, kd_pos, 
            TARGET_X_NORMALIZED,
            0.02f  // Sample time menor para permitir cálculos mais frequentes
        );
        
        this->circle_y_pid = std::make_unique<PidController>(
            kp_pos, ki_pos, kd_pos, 
            TARGET_Y_NORMALIZED,
            0.02f  // Sample time menor para permitir cálculos mais frequentes
        );

        // LOGS DE DEBUG PARA VERIFICAR PARÂMETROS
        this->drone->log("PID LATERAL - Kp: " + std::to_string(kp_lateral) + 
                        ", Ki: " + std::to_string(ki_lateral) + 
                        ", Kd: " + std::to_string(kd_lateral));
        this->drone->log("PID ANGULAR - Kp: " + std::to_string(kp_angular) + 
                        ", Ki: " + std::to_string(ki_angular) + 
                        ", Kd: " + std::to_string(kd_angular));
        this->drone->log("PID POSICIONAL - Kp: " + std::to_string(kp_pos) + 
                        ", Ki: " + std::to_string(ki_pos) + 
                        ", Kd: " + std::to_string(kd_pos));

        this->drone->log("FollowLaneState initialized with PID controllers");
    }

    std::string act(fsm::Blackboard& bb) override {
        (void) bb; // Evitar warning de parâmetro não usado

        auto lane_data = this->vision->getCurrentLaneData();
        bool circle_detected = lane_data.is_circle;

        // Reset PIDs quando mudamos de modo
        if (circle_detected != previous_circle_detected) {
            if (circle_detected) {
                // Mudamos para modo círculo - resetar PIDs do círculo
                this->circle_x_pid->reset();
                this->circle_y_pid->reset();
                this->drone->log("Switched to CIRCLE mode - PIDs reset");
            } else {
                // Mudamos para modo linha - resetar PIDs da linha
                this->lateral_pid->reset();
                this->angular_pid->reset();
                this->drone->log("Switched to LANE mode - PIDs reset");
            }
            previous_circle_detected = circle_detected;
        }

        float theta = lane_data.theta;

        int x_centroid_scaled = lane_data.x_centroid;
        float x_centroid_normalized = static_cast<float>(x_centroid_scaled) / 1000.0f;
        
        int y_centroid_scaled = lane_data.y_centroid;
        float y_centroid_normalized = static_cast<float>(y_centroid_scaled) / 1000.0f;

        if (abs(abs(drone->getAltitude()) - abs(height)) <= position_tolerance){
            if(!this->vision->isLaneDetected()) {
                this->drone->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f);
                this->drone->log("No lane detected - stopping");
                return "LANE ENDED";
            }
        }

        float lateral_correction = this->lateral_pid->compute(x_centroid_normalized);
        float angular_correction = this->angular_pid->compute(theta);
        
        // Normalizar correções para limites de velocidade
        lateral_correction = std::clamp(lateral_correction, -this->max_velocity, this->max_velocity);
        angular_correction = std::clamp(angular_correction, -this->max_yaw_rate, this->max_yaw_rate); // rad/s

        float vx = 0.0f, vy = 0.0f;
        float vz = 0.0f;                       // Manter altitude
        float yaw_rate = 0.0f;

        if (circle_detected) {
            this->drone->log("CIRCLE MODE");

            // Usar PID para controle posicional do círculo
            float circle_x_correction = this->circle_x_pid->compute(x_centroid_normalized);
            float circle_y_correction = this->circle_y_pid->compute(y_centroid_normalized);
            
            // Log dos valores ANTES do clamp
            this->drone->log("Raw PID values - X: " + std::to_string(circle_x_correction) + 
                           ", Y: " + std::to_string(circle_y_correction));
            
            // Se os valores de PID são 0, usar controle proporcional simples
            if (circle_x_correction == 0.0f && circle_y_correction == 0.0f) {
                circle_x_correction = -0.5f * x_centroid_normalized;  // Simples P control
                circle_y_correction = -0.5f * y_centroid_normalized;  // Simples P control
                this->drone->log("Using fallback P-control: X=" + std::to_string(circle_x_correction) + 
                               ", Y=" + std::to_string(circle_y_correction));
            }
            
            // Mapear correções para velocidades do drone
            // Y da imagem -> X do drone (movimento frente/trás) 
            // X da imagem -> Y do drone (movimento lateral)  
            vx = -circle_y_correction;
            vy = circle_x_correction;
            
            // Aplicar limites de velocidade
            vx = std::clamp(vx, -this->max_velocity, this->max_velocity);
            vy = std::clamp(vy, -this->max_velocity, this->max_velocity);
            
            yaw_rate = 0.0f; // NÃO rotacionar quando detectar círculo
            
            // Check se está centrado o suficiente
            if (std::abs(x_centroid_normalized) < this->position_tolerance && 
                std::abs(y_centroid_normalized) < this->position_tolerance) {
                this->drone->log("Circle detected and centered - stopping drone");
                this->drone->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f);
                return "CIRCLE DETECTED";
            }
            
            // Log específico para círculo
            this->drone->log("Following [CIRCLE]" +
                            std::string(", X_norm: ") + std::to_string(x_centroid_normalized) +
                            ", Y_norm: " + std::to_string(y_centroid_normalized) +
                            ", PID_X: " + std::to_string(circle_x_correction) +
                            ", PID_Y: " + std::to_string(circle_y_correction) +
                            ", Vx: " + std::to_string(vx) +
                            ", Vy: " + std::to_string(vy));
            
        } else {
            this->drone->log("LANE MODE");

            bool angle_too_large = std::abs(std::abs(theta) - 1.57) > this->max_angle_for_translation;

            if(angle_too_large) {
                this->drone->log("Angle too large for translation - rotating only");
                yaw_rate = angular_correction; // Apenas rotacionar
            }
            else {
                this->drone->log("Following lane with translation and rotation");
                vx = this->max_velocity;          // Para frente
                vy = -lateral_correction;         // Correção lateral
                yaw_rate = angular_correction;    // Correção angular
            }
            
            // Log específico para lane
            this->drone->log("Following [LANE]" +
                            std::string(", Theta: ") + std::to_string(theta) +
                            ", Vx: " + std::to_string(vx) +
                            ", Vy: " + std::to_string(vy) +
                            ", Yaw_rate: " + std::to_string(yaw_rate));
        }

        // Comandar velocidades no frame local do drone
        Eigen::Vector3d local_velocity(vx, vy, vz);
        local_velocity = adjust_velocity_using_yaw(local_velocity, this->drone->getOrientation()[2]); // o ultimo parametro é o yaw do drone
        this->drone->setLocalVelocity(local_velocity.x(), local_velocity.y(), local_velocity.z(), yaw_rate);

        return "";
    }

    void on_exit(fsm::Blackboard& bb) override {
        (void) bb;
        // Parar o drone quando sair do estado
        if (this->drone != nullptr) {
            this->drone->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f);
            this->drone->log("Exiting FollowLaneState - stopping drone");
        }
    }
};