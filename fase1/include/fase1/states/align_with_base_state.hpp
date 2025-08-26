#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "fase1/vision_fase1.hpp"
#include "fase1/ArenaPoint.hpp"
#include "fase1/movement.hpp"
#include "fase1/PidController.hpp"

class AlignWithBaseState : public fsm::State {
private:
    std::shared_ptr<Drone> drone;
    std::shared_ptr<VisionNode> vision;

    std::unique_ptr<PidController> x_pid;
    std::unique_ptr<PidController> y_pid;

    std::chrono::steady_clock::time_point start_time_;
    
    ArenaPoint waypoint;
    
    float max_velocity;
    float position_tolerance;
    float timeout_duration;

    static constexpr float SETPOINT_X = 0.0f;  // Centro da imagem (0 = centro)
    static constexpr float SETPOINT_Y = 0.0f;  // Centro da imagem (0 = centro)

public:
    AlignWithBaseState() : fsm::State(), waypoint("default", 0.0f, 0.0f, 0.0f) {}  // Initialize waypoint



    void on_enter(fsm::Blackboard &bb) override {
        auto drone_ptr = bb.get<std::shared_ptr<Drone>>("drone");
        auto vision_ptr = bb.get<std::shared_ptr<VisionNode>>("vision");

        if(drone_ptr == nullptr || vision_ptr == nullptr) return;

        this->drone = *drone_ptr;
        this->vision = *vision_ptr;

        this->drone->log("STATE: ALIGN WITH BASE");

        this->max_velocity = *bb.get<float>("max_vertical_velocity");
        this->position_tolerance = *bb.get<float>("position_tolerance");

        this->timeout_duration = 10.0f;  // Use direct value since not in parameters

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

        this->drone->setLocalVelocity(0.0, 0.0, 0.0);
        this->start_time_ = std::chrono::steady_clock::now();

    }



    std::string act(fsm::Blackboard &bb) override {
        (void) bb;

        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - this->start_time_).count();
        if(elapsed > this->timeout_duration){
            this->drone->setLocalVelocity(0.0, 0.0, 0.0);
            this->drone->log("Timeout ao alinhar com a base");
            return "TIMEOUT";
        }

        auto base = *bb.get<Base>("base_detected");
        float x_base_ref_drone = -1*((base.y - 0.5f) * 2.0f); // converter de [0,1] para [-1,1], pois o centro é 0.5
        float y_base_ref_drone = (base.x - 0.5f) * 2.0f; // converter de [0,1] para [-1,1], pois o centro é 0.5

        //auto pos = this->drone->getLocalPosition();

        float error_x = x_base_ref_drone - SETPOINT_X;
        float error_y = y_base_ref_drone - SETPOINT_Y;

        if(abs(error_x) < this->position_tolerance &&
           abs(error_y) < this->position_tolerance){
            this->drone->setLocalVelocity(0.0, 0.0, 0.0);
            this->drone->log("Alinhado com a base!!");
            return "ALIGNED";
        }

        float vx = this->x_pid->compute(x_base_ref_drone);
        float vy = this->y_pid->compute(y_base_ref_drone);

        vx = std::clamp(vx, -this->max_velocity, this->max_velocity);
        vy = std::clamp(vy, -this->max_velocity, this->max_velocity);
        
        move_local(this->drone, vx, vy, 0.0f);

        this->drone->log("x_error: " + std::to_string(error_x) + ", y_error: " + std::to_string(error_y));

        return "";
    }

private:
    // This method is not used in this state, remove it or fix if needed
    /*
    ArenaPoint getNextCheckpoint(){
        // This method would need waypoints member which doesn't exist here
        return ArenaPoint("dummy", 0.0f, 0.0f, 0.0f);        
    }
    */
};