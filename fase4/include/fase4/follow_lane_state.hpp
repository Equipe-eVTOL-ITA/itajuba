#include <Eigen/Eigen>
#include <chrono>
#include <algorithm>
#include <cmath>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "vision_fase4.hpp"
#include "fase4/PidController.hpp"

class FollowLaneState : public fsm::State {
private:
    std::shared_ptr<Drone> drone;
    std::shared_ptr<VisionNode> vision;
    
    // PID Controllers
    std::unique_ptr<PidController> lateral_pid;  // Para alinhamento X
    std::unique_ptr<PidController> angular_pid;  // Para alinhamento do theta (yaw)
    
    // Parâmetros
    float max_velocity;
    float position_tolerance;
    float height;
    float forward_velocity; // Velocidade constante para frente
    
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

        this->drone->log("FollowLaneState initialized with PID controllers");
    }

    std::string act(fsm::Blackboard& bb) override {
        (void) bb; // Evitar warning de parâmetro não usado
        
        // Verificar se temos detecção de lane
        if (!this->vision->isLaneDetected()) {
            // Se não há detecção, parar o drone
            this->drone->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f);
            this->drone->log("No lane detected - stopping");
            return "LANE_LOST";
        }

        // Obter dados da lane detection
        auto lane_data = this->vision->getCurrentLaneData();
        float theta = lane_data.theta;
        int x_centroid_scaled = lane_data.x_centroid;
        
        // Converter coordenadas normalizadas de volta para float
        // (foram multiplicadas por 1000 no Python para manter precisão como int)
        float x_centroid_normalized = static_cast<float>(x_centroid_scaled) / 1000.0f;
        
        // Calcular correções usando PIDs (agora com coordenadas normalizadas)
        float lateral_correction = this->lateral_pid->compute(x_centroid_normalized);
        float angular_correction = this->angular_pid->compute(theta);
        
        // Normalizar correções para limites de velocidade
        lateral_correction = std::clamp(lateral_correction, -this->max_velocity, this->max_velocity);
        angular_correction = std::clamp(angular_correction, -1.0f, 1.0f); // rad/s
        
        // Comandar velocidades:
        // vx: movimento para frente (constante)
        // vy: correção lateral (baseada no erro de posição X normalizada)
        // vz: manter altitude (0 para manter altura atual)
        // yaw_rate: correção angular (baseada no erro de theta)
        float vx = this->max_velocity;  // Movimento para frente
        float vy = -lateral_correction;     // Correção lateral (negativo para corrigir)
        float vz = 0.0f;                   // Manter altitude
        float yaw_rate = angular_correction; // Correção de orientação
        
        this->drone->setLocalVelocity(vx, vy, vz, yaw_rate);
        
        // Log para debug (agora com coordenadas normalizadas)
        this->drone->log("Lane following - X_norm: " + std::to_string(x_centroid_normalized) + 
                        ", X_error: " + std::to_string(x_centroid_normalized - TARGET_X_NORMALIZED) +
                        ", Theta: " + std::to_string(theta) +
                        ", Lateral_cmd: " + std::to_string(vy) +
                        ", Angular_cmd: " + std::to_string(yaw_rate));

        // Verificar condições de saída (pode ser implementado conforme necessário)
        // Talvez funcione analisar o tamanho da area detectada ou tempo sem detecção
        
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