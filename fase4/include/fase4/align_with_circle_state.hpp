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
    float normalized_position_tolerance;
    float timeout_duration;
    float height;
    
    // centro da imagem
    static constexpr float SETPOINT_X = 0.0f;
    static constexpr float SETPOINT_Y = 0.0f;

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
        this->normalized_position_tolerance = *bb.get<float>("normalized_position_tolerance_circle_align");
        this->height = *bb.get<float>("takeoff_height");

        this->timeout_duration = *bb.get<float>("timeout_circle_detection");

        float kp_pos = *bb.get<float>("pid_pos_kp");
        float ki_pos = *bb.get<float>("pid_pos_ki");
        float kd_pos = *bb.get<float>("pid_pos_kd");
        
        this->x_pid = std::make_unique<PidController>(
            kp_pos, ki_pos, kd_pos, 
            SETPOINT_X
        );
        
        this->y_pid = std::make_unique<PidController>(
            kp_pos, ki_pos, kd_pos, 
            SETPOINT_Y
        );

        this->drone->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f); // parar movimento anterior
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

        float x_centroid_normalized;
        float y_centroid_normalized;

        if(!lane_data.is_circle){
            x_centroid_normalized = *bb.get<float>("last_x_circle") / 1000.0f;
            y_centroid_normalized = *bb.get<float>("last_y_circle") / 1000.0f;
        }
        else {
            x_centroid_normalized = static_cast<float>(lane_data.x_centroid) / 1000.0f;
            y_centroid_normalized = static_cast<float>(lane_data.y_centroid) / 1000.0f;
        }

        // Logar valores normalizados recebidos
        this->drone->log("x_centroid_normalized: " + std::to_string(x_centroid_normalized) + ", y_centroid_normalized: " + std::to_string(y_centroid_normalized));

        float drone_x_coord = -y_centroid_normalized;
        float drone_y_coord = x_centroid_normalized;

        float error_drone_x = drone_x_coord - SETPOINT_X;
        float error_drone_y = drone_y_coord - SETPOINT_Y;

        if(abs(error_drone_x) < normalized_position_tolerance && abs(error_drone_y) < normalized_position_tolerance) {
            float distance_to_target = std::sqrt(error_drone_x * error_drone_x + error_drone_y * error_drone_y);
            this->drone->log("SUCCESS: Aligned with circle (distance: " + std::to_string(distance_to_target) + ")");
            return "ALIGNED";
        }

        // Inverter sinal do PID se necessário
        float vel_x = -this->x_pid->compute(drone_x_coord);
        float vel_y = -this->y_pid->compute(drone_y_coord);

        vel_x = std::clamp(vel_x, -max_velocity, max_velocity);
        vel_y = std::clamp(vel_y, -max_velocity, max_velocity);

        float vz = 0.0f; // manter altura constante

        Eigen::Vector3d local_velocity(vel_x, vel_y, vz);
        local_velocity = adjust_velocity_using_yaw(local_velocity, this->drone->getOrientation()[2]); // o ultimo parametro é o yaw do drone
        this->drone->setLocalVelocity(local_velocity.x(), local_velocity.y(), local_velocity.z(), 0.0f); // Não rotacionar durante alinhamento

        this->drone->log("drone_x_coord: " + std::to_string(drone_x_coord) + ", drone_y_coord: " + std::to_string(drone_y_coord));

        return "";
    }

};