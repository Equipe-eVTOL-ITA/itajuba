#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

enum TAKEOFF_PHASE {
    INITIAL_TAKEOFF = 0,
    GOAL_TAKEOFF = 1
};

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

        this->take_off_taken = *bb.get<bool>("initial_takeoff_taken");

        this->log_(true);
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

            if(!this->take_off_taken) {
                bb.set<bool>("initial_takeoff_taken", true);
                return "INITIAL TAKEOFF COMPLETED"; // proximo é comecar a procurar a pista para depois segui-la
            }
            
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

private:
    void log_(bool initial_yaw) {
        this->drone->log("Takeoff initiated at position: " + 
                         std::to_string(this->pos[0]) + ", " + 
                         std::to_string(this->pos[1]) + ", " + 
                         std::to_string(this->pos[2]));
        if(initial_yaw)
            this->drone->log("Initial Yaw: " + std::to_string(this->initial_yaw));
    }
};