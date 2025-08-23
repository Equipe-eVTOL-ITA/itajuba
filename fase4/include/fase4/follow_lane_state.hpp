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
    
    std::unique_ptr<PidController> lateral_pid;  // Para alinhamento em x
    std::unique_ptr<PidController> angular_pid;  // Para alinhamento do theta (yaw)
    
    float max_velocity;
    float max_yaw_rate;
    float position_tolerance;
    float height;
    float max_angle_for_translation;
    float angulo_muito_pequeno_seguir_reto;
    
    // Setpoints para controle
    static constexpr float SETPOINT_X = 0.0f;  // Centro da imagem (0 = centro)
    static constexpr float SETPOINT_Y = 0.0f;  // Centro da imagem (0 = centro)
    static constexpr float SETPOINT_THETA = 0.0f;         // Theta alvo (vertical) 

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

        this->angulo_muito_pequeno_seguir_reto = *bb.get<float>("angulo_muito_pequeno_seguir_reto");
        this->max_angle_for_translation = *bb.get<float>("max_angle_for_translation");
        this->position_tolerance = *bb.get<float>("position_tolerance");
        this->max_velocity = *bb.get<float>("max_horizontal_velocity");
        this->max_yaw_rate = *bb.get<float>("max_yaw_rate");
        this->height = *bb.get<float>("takeoff_height");


        float kp_lateral = *bb.get<float>("pid_lateral_kp");
        float ki_lateral = *bb.get<float>("pid_lateral_ki"); 
        float kd_lateral = *bb.get<float>("pid_lateral_kd");
        
        float kp_angular = *bb.get<float>("pid_angular_kp");
        float ki_angular = *bb.get<float>("pid_angular_ki");
        float kd_angular = *bb.get<float>("pid_angular_kd");
        
        this->lateral_pid = std::make_unique<PidController>(
            kp_lateral, ki_lateral, kd_lateral, 
            SETPOINT_X
        );
        
        this->angular_pid = std::make_unique<PidController>(
            kp_angular, ki_angular, kd_angular, 
            SETPOINT_THETA
        );

        this->drone->log("FollowLaneState initialized!");
    }

    std::string act(fsm::Blackboard& bb) override {
        (void) bb; // Evitar warning de parâmetro não usado

        auto lane_data = this->vision->getCurrentLaneData();

        if(lane_data.is_circle){
            bb.set<bool>("has_ever_detected_circle", true);
            bb.set<float>("last_x_circle", lane_data.x_centroid);
            bb.set<float>("last_y_circle", lane_data.y_centroid);
            return "CIRCLE DETECTED";
        }


        float theta = lane_data.theta; // entre -pi/2 e pi/2, mas pi/2 e -pi/2 são a mesma direção (vertical)

        int x_centroid_scaled = lane_data.x_centroid;
        int y_centroid_scaled = lane_data.y_centroid;
        
        float x_centroid_normalized = static_cast<float>(x_centroid_scaled) / 1000.0f;
        float y_centroid_normalized = static_cast<float>(y_centroid_scaled) / 1000.0f;

        bool voltando = *bb.get<bool>("has_ever_detected_circle");
        if(voltando) {
            this->drone->log("Y: "+std::to_string(y_centroid_normalized));
            if(y_centroid_normalized > 0.7f){ // controlar essa porcentagem faz escolher quando pousar certinho na base
                bb.set<bool>("has_ever_detected_base", true);
                return "BASE DETECTED";
            }
        }

        if(!this->vision->isLaneDetected()) {
            this->drone->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f);
            this->drone->log("Não encontrei a faixa...");
            return "LANE LOST";
        }

        float lateral_correction = this->lateral_pid->compute(x_centroid_normalized);
        float angular_correction = this->angular_pid->compute(theta);
        
        // Limitando as velocidades de correção
        lateral_correction = std::clamp(lateral_correction, -this->max_velocity, this->max_velocity);
        angular_correction = std::clamp(angular_correction, -this->max_yaw_rate, this->max_yaw_rate); // rad/s

        float vx = 0.0f, vy = 0.0f;
        float vz = 0.0f; // manter altitude
        float yaw_rate = 0.0f;

        this->drone->log("LANE MODE");

        bool angle_too_large = std::abs(std::abs(theta) - 1.57) > this->max_angle_for_translation;

        if(angle_too_large) {
            this->drone->log("Apenas rotacionando, angulo muito grande para controle linear");
            yaw_rate = angular_correction; // Apenas rotacionar
        }
        else {
            this->drone->log("Transladando e rotacionando");
            vx = this->max_velocity; // frente
            vy = -lateral_correction;
            yaw_rate = angular_correction;
        }

        if(std::abs(theta) < this->angulo_muito_pequeno_seguir_reto) {
            this->drone->log("Situação estranha encontrada (angulo muito pequeno): " + std::to_string(theta));
            vx = 0.1f; // Seguir reto com velocidade baixa
            vy = 0.0f;
            yaw_rate = 0.0f; // Não rotacionar
        }

        // Passando o comando usando o frame local do drone
        Eigen::Vector3d local_velocity(vx, vy, vz);
        local_velocity = adjust_velocity_using_yaw(local_velocity, this->drone->getOrientation()[2]); // o ultimo parametro é o yaw do drone
        this->drone->setLocalVelocity(local_velocity.x(), local_velocity.y(), local_velocity.z(), yaw_rate);

        return "";
    }

    void on_exit(fsm::Blackboard& bb) override {
        (void) bb;
        if (this->drone != nullptr) {
            this->drone->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f);
            this->drone->log("Exiting FollowLaneState...");
        }
    }
};