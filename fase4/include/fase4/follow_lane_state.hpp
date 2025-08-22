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
    
    // Parâmetros
    float max_velocity;
    float max_yaw_rate;
    float position_tolerance;
    float height;
    
    // Setpoints para controle (coordenadas normalizadas)
    static constexpr float TARGET_X_NORMALIZED = 0.0f;  // Centro da imagem (0 = centro)
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

        // Inicializar PIDs
        // PID lateral: setpoint = 0 (centro da imagem normalizado)
        float kp_lateral = *bb.get<float>("pid_lateral_kp");
        float ki_lateral = *bb.get<float>("pid_lateral_ki"); 
        float kd_lateral = *bb.get<float>("pid_lateral_kd");
        
        // PID angular: setpoint = 0 (theta vertical)
        float kp_angular = *bb.get<float>("pid_angular_kp");
        float ki_angular = *bb.get<float>("pid_angular_ki");
        float kd_angular = *bb.get<float>("pid_angular_kd");
        
        this->lateral_pid = std::make_unique<PidController>(
            kp_lateral, ki_lateral, kd_lateral, 
            TARGET_X_NORMALIZED
        );
        
        this->angular_pid = std::make_unique<PidController>(
            kp_angular, ki_angular, kd_angular, 
            TARGET_THETA
        );

        // LOGS DE DEBUG PARA VERIFICAR PARÂMETROS
        this->drone->log("PID LATERAL - Kp: " + std::to_string(kp_lateral) + 
                        ", Ki: " + std::to_string(ki_lateral) + 
                        ", Kd: " + std::to_string(kd_lateral));
        this->drone->log("PID ANGULAR - Kp: " + std::to_string(kp_angular) + 
                        ", Ki: " + std::to_string(ki_angular) + 
                        ", Kd: " + std::to_string(kd_angular));

        this->drone->log("FollowLaneState initialized with PID controllers");
    }

    std::string act(fsm::Blackboard& bb) override {
        (void) bb; // Evitar warning de parâmetro não usado

        auto lane_data = this->vision->getCurrentLaneData();
        bool circle_detected = lane_data.is_circle;

        float theta = lane_data.theta;

        int x_centroid_scaled = lane_data.x_centroid;
        float x_centroid_normalized = static_cast<float>(x_centroid_scaled) / 1000.0f;
        
        //int y_centroid_scaled = lane_data.y_centroid;
        //float y_centroid_normalized = static_cast<float>(y_centroid_scaled) / 1000.0f;

        if (abs(abs(drone->getAltitude()) - abs(height)) <= position_tolerance){
            if(circle_detected){
                this->drone->log("Circle detected - stopping drone");
                this->drone->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f);
                return "CIRCLE DETECTED";
            }
            else if(!this->vision->isLaneDetected()) {
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

        float vx = this->max_velocity;          // Sempre para frente
        float vy = -lateral_correction;
        float vz = 0.0f;                       // Manter altitude
        float yaw_rate = angular_correction;
        
        // Comandar velocidades no frame local do drone
        Eigen::Vector3d local_velocity(vx, vy, vz);
        local_velocity = adjust_velocity_using_yaw(local_velocity, this->drone->getOrientation()[2]); // o ultimo parametro é o yaw do drone
        this->drone->setLocalVelocity(local_velocity.x(), local_velocity.y(), local_velocity.z(), yaw_rate);

        // Log para debug simples
        this->drone->log("Lane following - X_norm: " + std::to_string(x_centroid_normalized) + 
                        ", X_error: " + std::to_string(x_centroid_normalized - TARGET_X_NORMALIZED) +
                        ", Theta: " + std::to_string(theta) +
                        ", Vx: " + std::to_string(vx) +
                        ", Vy: " + std::to_string(vy) +
                        ", Yaw_rate: " + std::to_string(yaw_rate));

        // Verificar condições de saída
        // Talvez funcione analisar o tamanho da area detectada ou tempo sem detecção
        // talvez tambem ajustar a velocidade maxima de acordo coma area

        return ""; // Continuar no estado
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