#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "fase1/vision_fase1.hpp"
#include "fase1/ArenaPoint.hpp"

class SearchBaseState : public fsm::State {
private:
    std::shared_ptr<Drone> drone;
    std::shared_ptr<VisionNode> vision;  // Fixed typo: vison -> vision

    std::vector<ArenaPoint> waypoints;

    Eigen::Vector3d pos;
    Eigen::Vector3d goal;
    
    float max_velocity;
    float position_tolerance;
    float initial_yaw;

public:
    SearchBaseState() : fsm::State() {}



    void on_enter(fsm::Blackboard &bb) override {
        auto drone_ptr = bb.get<std::shared_ptr<Drone>>("drone");
        auto vision_ptr = bb.get<std::shared_ptr<VisionNode>>("vision");

        if(drone_ptr == nullptr || vision_ptr == nullptr) return;

        this->drone = *drone_ptr;
        this->vision = *vision_ptr;

        this->drone->log("STATE: SEARCH BASE");

        this->max_velocity = *bb.get<float>("max_vertical_velocity");
        this->position_tolerance = *bb.get<float>("position_tolerance");

        this->pos = this->drone->getLocalPosition();
        this->initial_yaw = this->drone->getOrientation()[2];

        this->waypoints = *bb.get<std::vector<ArenaPoint>>("waypoints");
    }



    std::string act(fsm::Blackboard &bb) override {
        (void) bb;

        if(this->vision->hasValidBase()){
            this->drone->log("Base detectada");
            this->drone->setLocalVelocity(0.0, 0.0, 0.0);
            bb.set<Base>("base_detected", this->vision->getCurrentBase());
            return "BASE DETECTED";
        }

        this->pos = this->drone->getLocalPosition();

        ArenaPoint* p = this->getNextCheckpoint(); // Get pointer to modify is_visited
        if(p == nullptr){
            this->drone->log("Acabaram os waypoints");
            return "SEARCH ENDED";
        }
        this->goal = p->coord;

        Eigen::Vector3d diff = this->goal - this->pos;

        if(diff.norm() < this->position_tolerance) {
            this->drone->log("Cheguei no checkpoint: "+p->name);
            p->is_visited = true;
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
    ArenaPoint* getNextCheckpoint(){
        for(auto& p : this->waypoints){  // Use reference to modify
            if(p.is_visited == false){
                return &p;  // Return pointer
            }
        }
        return nullptr;        
    }
};