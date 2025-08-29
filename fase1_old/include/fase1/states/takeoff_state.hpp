#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

class TakeoffState : public fsm::State {
private:
    std::shared_ptr<Drone> drone;
    Eigen::Vector3d pos;
    Eigen::Vector3d goal;
    float max_velocity;
    float position_tolerance;
    float initial_yaw;
    bool take_off_taken;

public:
    TakeoffState() : fsm::State() {}



    void on_enter(fsm::Blackboard &bb) override {
        this->drone = *bb.get<std::shared_ptr<Drone>>("drone");
        
        if(this->drone == nullptr) return;
        
        this->drone->log("STATE: TAKEOFF");

        this->max_velocity = *bb.get<float>("max_vertical_velocity");
        this->position_tolerance = *bb.get<float>("position_tolerance");

        float takeoff_height = *bb.get<float>("takeoff_height");

        this->pos = this->drone->getLocalPosition();
        this->initial_yaw = this->drone->getOrientation()[2];
        this->goal = Eigen::Vector3d({this->pos[0], this->pos[1], takeoff_height});
    }



    std::string act(fsm::Blackboard &bb) override {
        (void) bb;

        this->pos = this->drone->getLocalPosition();
        Eigen::Vector3d diff = this->goal - this->pos;

        if(diff.norm() < this->position_tolerance) {
            this->drone->log("Takeoff completed at position: " + 
                             std::to_string(this->pos[0]) + ", " + 
                             std::to_string(this->pos[1]) + ", " + 
                             std::to_string(this->pos[2]));
            
            return "TAKEOFF COMPLETED"; // proximo é o volver
        }

        Eigen::Vector3d little_goal =
            pos + (diff.norm() > max_velocity ? diff.normalized() * max_velocity : diff);
        
        this->drone->setLocalPosition(
            little_goal.x(),
            little_goal.y(),
            little_goal.z(),
            this->initial_yaw
        );

        return "";
    }
};