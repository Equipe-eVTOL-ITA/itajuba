#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "fase1/ArenaPoint.hpp"
#include <chrono>

class ReturnHomeState : public fsm::State {
private:
    std::shared_ptr<Drone> drone;
    std::vector<ArenaPoint> waypoints;
    
    Eigen::Vector3d pos;
    Eigen::Vector3d goal;
    
    float max_velocity;
    float position_tolerance;
    float initial_yaw;

public:
    ReturnHomeState() {}

    void on_enter(fsm::Blackboard &bb) override {
        auto drone_ptr = bb.get<std::shared_ptr<Drone>>("drone");

        if(drone_ptr == nullptr) return;

        this->drone = *drone_ptr;

        this->drone->log("STATE: RETURN HOME STATE");
        
        this->max_velocity = *bb.get<float>("max_vertical_velocity");
        this->position_tolerance = *bb.get<float>("position_tolerance");
        
        this->pos = this->drone->getLocalPosition();
        this->initial_yaw = this->drone->getOrientation()[2];
        
        
        this->waypoints = *bb.get<std::vector<ArenaPoint>>("waypoints");
        for(const auto& waypoint : this->waypoints) {
            if(waypoint.name == "Origin") {
                this->goal = waypoint.coord;
                break;
            }
        }
    }

    std::string act(fsm::Blackboard &bb) override {
        (void) bb;
        
        this->pos = this->drone->getLocalPosition();
        
        Eigen::Vector3d diff = this->goal - this->pos;
        
        if(diff.norm() < this->position_tolerance) {
            this->drone->log("Retorno à Origem concluído.");
            this->drone->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
            return "RETURNED";
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