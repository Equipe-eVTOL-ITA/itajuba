#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "fase4/PidController.hpp"
#include "fase4/transformations.hpp"
#include "vision_fase4.hpp"
#include <chrono>
#include <cmath>

class AlignWithCircleState : public fsm::State {
private:
    std::shared_ptr<Drone> drone;
    std::shared_ptr<VisionNode> vision;

    std::unique_ptr<PidController> x_pid;
    std::unique_ptr<PidController> y_pid;
    
    std::chrono::steady_clock::time_point start_time_;
    
    float max_velocity;
    float position_tolerance;
    float timeout_duration;
    float height;
    
    // centro da imagem
    static constexpr float TARGET_X_NORMALIZED = 0.0f;  // Centro da imagem (0 = centro)
    static constexpr float TARGET_Y_NORMALIZED = 0.0f;  // Centro da imagem (0 = centro)

public:
    AlignWithCircleState() : fsm::State() {}

    void on_enter(fsm::Blackboard &bb) override {
        auto drone_ptr = bb.get<std::shared_ptr<Drone>>("drone");
        auto vision_ptr = bb.get<std::shared_ptr<VisionNode>>("vision");

        if(drone_ptr == nullptr || *drone_ptr == nullptr){
            this->drone->log("ERROR: Drone pointer is null in AlignWithCircleState");
            return;
        }
        
        if(vision_ptr == nullptr || *vision_ptr == nullptr){
            this->drone->log("ERROR: Vision pointer is null in AlignWithCircleState");
            return;
        }

        this->drone = *drone_ptr;
        this->vision = *vision_ptr;

        this->drone->log("STATE: ALIGN WITH CIRCLE STATE");

        this->start_time_ = std::chrono::steady_clock::now();
        
        this->max_velocity = *bb.get<float>("max_horizontal_velocity");
        this->position_tolerance = *bb.get<float>("position_tolerance");
        this->height = *bb.get<float>("takeoff_height");
        
        // Tentar obter timeout específico, senão usar padrão
        auto timeout_ptr = bb.get<float>("timeout");
        this->timeout_duration = timeout_ptr ? *timeout_ptr : 10.0f;
        
        // Inicializar PIDs para alinhamento
        // IMPORTANTE: Mapeamento de coordenadas imagem -> drone:
        // - x_pid: controla movimento X do drone (frente/trás) baseado no erro Y da imagem (cima/baixo)
        // - y_pid: controla movimento Y do drone (esquerda/direita) baseado no erro X da imagem (esquerda/direita)
        float kp_pos = *bb.get<float>("pid_pos_kp");
        float ki_pos = *bb.get<float>("pid_pos_ki");
        float kd_pos = *bb.get<float>("pid_pos_kd");
        
        this->x_pid = std::make_unique<PidController>(
            kp_pos, ki_pos, kd_pos, 
            TARGET_X_NORMALIZED  // Target sempre 0 (centro)
        );
        
        this->y_pid = std::make_unique<PidController>(
            kp_pos, ki_pos, kd_pos, 
            TARGET_Y_NORMALIZED  // Target sempre 0 (centro)
        );
        
        this->drone->log("AlignWithCircleState initialized with PID controllers");
        this->drone->log("PID Gains - Kp: " + std::to_string(kp_pos) + 
                        ", Ki: " + std::to_string(ki_pos) + 
                        ", Kd: " + std::to_string(kd_pos));
    }

    std::string act(fsm::Blackboard &bb) override {
        (void) bb;

        // Verificar timeout
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time_).count();
        
        if (elapsed_time > timeout_duration) {
            this->drone->log("TIMEOUT: AlignWithCircle state exceeded maximum time");
            return "TIMEOUT";
        }

        auto lane_data = this->vision->getCurrentLaneData();
        
        // Converter coordenadas para valores normalizados
        float x_centroid_normalized = static_cast<float>(lane_data.x_centroid) / 1000.0f;
        float y_centroid_normalized = static_cast<float>(lane_data.y_centroid) / 1000.0f;

        float error_x_image = x_centroid_normalized - TARGET_X_NORMALIZED;
        float error_y_image = y_centroid_normalized - TARGET_Y_NORMALIZED;
        
        float error_drone_x = -error_y_image;
        float error_drone_y = error_x_image;
        
        float distance_to_target = std::sqrt(error_drone_x * error_drone_x + error_drone_y * error_drone_y);
        
        if (distance_to_target < position_tolerance && lane_data.is_circle) {
            this->drone->log("SUCCESS: Aligned with circle (distance: " + std::to_string(distance_to_target) + ")");
            return "ALIGNED";
        }
        
        float drone_x_coord = -y_centroid_normalized;
        float drone_y_coord = x_centroid_normalized;
        
        float vel_x = this->x_pid->compute(drone_x_coord);
        float vel_y = this->y_pid->compute(drone_y_coord);
        
        vel_x = std::clamp(vel_x, -max_velocity, max_velocity);
        vel_y = std::clamp(vel_y, -max_velocity, max_velocity);
        
        float vz = 0.0f; // Manter altura constante
        
        Eigen::Vector3d local_velocity(vel_x, vel_y, vz);
        local_velocity = adjust_velocity_using_yaw(local_velocity, this->drone->getOrientation()[2]); // o ultimo parametro é o yaw do drone
        this->drone->setLocalVelocity(local_velocity.x(), local_velocity.y(), local_velocity.z(), 0.0f); // Não rotacionar durante alinhamento
        
        this->drone->log("Aligning - Image X: " + std::to_string(x_centroid_normalized) + 
                        ", Image Y: " + std::to_string(y_centroid_normalized) + 
                        ", Drone X coord: " + std::to_string(drone_x_coord) + 
                        ", Drone Y coord: " + std::to_string(drone_y_coord) + 
                        ", Distance: " + std::to_string(distance_to_target) + 
                        ", Vel X: " + std::to_string(vel_x) + 
                        ", Vel Y: " + std::to_string(vel_y) +
                        ", Local Vel X: " + std::to_string(local_velocity.x()) + 
                        ", Local Vel Y: " + std::to_string(local_velocity.y()));
        
        return "";
    }

};