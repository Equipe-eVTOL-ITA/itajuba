#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "vision_fase2.hpp"
#include "fase2/comandos.hpp"
#include "PidController.hpp"

class ArucoAlignState : public fsm::State {
private:
    std::shared_ptr<Drone> drone;
    std::shared_ptr<VisionNode> vision;

    std::unique_ptr<PidController> x_pid;
    std::unique_ptr<PidController> y_pid;

    std::chrono::steady_clock::time_point start_time_;

    ArucoMarker marker;

    float timeout_duration;
    float normalized_position_tolerance;
    float max_velocity;
    float height;

    // centro da imagem
    static constexpr float SETPOINT_X = 0.0f;
    static constexpr float SETPOINT_Y = 0.0f;

public:
    ArucoAlignState() : fsm::State() {}



    void on_enter(fsm::Blackboard &bb) override {
        auto drone_ptr = bb.get<std::shared_ptr<Drone>>("drone");
        auto vision_ptr = bb.get<std::shared_ptr<VisionNode>>("vision");

        if(drone_ptr == nullptr || vision_ptr == nullptr) return;

        this->drone = *drone_ptr;
        this->vision = *vision_ptr;

        this->drone->log("STATE: ARUCO ALIGN");

        this->max_velocity = *bb.get<float>("max_horizontal_velocity");
        this->height = *bb.get<float>("takeoff_height");

        this->timeout_duration = *bb.get<float>("timeout_aruco_align");
        this->normalized_position_tolerance = *bb.get<float>("normalized_position_tolerance");
    
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
        this->start_time_ = std::chrono::steady_clock::now();
    }



    std::string act(fsm::Blackboard &bb) override {
        (void) bb;

        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - this->start_time_).count();
        if(elapsed_time > this->timeout_duration){
            this->drone->log("ERROR: Timeout in Aruco Align State");
            return "TIMEOUT";
        }
        
        this->marker = this->vision->getCurrentMarker();

        if(this->marker.dir == Direcoes::NENHUMA){
            this->drone->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f); // parar movimento anterior
            return "NO MARKER";
        }

        // Normalizar coordenadas do marcador
        float x_marker_normalized = this->marker.x / 100.0f;
        float y_marker_normalized = this->marker.y / 100.0f;

        // Logar valores normalizados recebidos
        this->drone->log("x_marker_normalized: " + std::to_string(x_marker_normalized) + ", y_marker_normalized: " + std::to_string(y_marker_normalized));

        float drone_x_coord = -y_marker_normalized;
        float drone_y_coord = x_marker_normalized;

        float error_drone_x = drone_x_coord - SETPOINT_X;
        float error_drone_y = drone_y_coord - SETPOINT_Y;

        if(abs(error_drone_x) < this->normalized_position_tolerance && 
           abs(error_drone_y) < this->normalized_position_tolerance){
            this->drone->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f); // parar movimento anterior
            this->drone->log("Aruco Aligned! (distance: " + std::to_string(std::sqrt(error_drone_x * error_drone_x + error_drone_y * error_drone_y)) + ")");
            return "ALIGNED";
        }

        float vel_x = -this->x_pid->compute(drone_x_coord);
        float vel_y = -this->y_pid->compute(drone_y_coord);
        
        vel_x = std::clamp(vel_x, -this->max_velocity, this->max_velocity);
        vel_y = std::clamp(vel_y, -this->max_velocity, this->max_velocity);

        Eigen::Vector3d local_velocity(vel_x, vel_y, 0.0f);
        local_velocity = adjust_velocity_using_yaw(local_velocity, this->drone->getOrientation()[2]);
        this->drone->setLocalVelocity(local_velocity.x(), local_velocity.y(), local_velocity.z(), 0.0f);

        this->drone->log("drone_x_coord: " + std::to_string(drone_x_coord) + ", drone_y_coord: " + std::to_string(drone_y_coord));

        return "";
    }
};