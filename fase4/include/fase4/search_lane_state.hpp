#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "vision_fase4.hpp"
#include "fase4/PidController.hpp"
#include "fase4/transformations.hpp"

class SearchLaneState : public fsm::State {
private:
    std::shared_ptr<Drone> drone;
    std::shared_ptr<VisionNode> vision;
    float max_velocity;

public:
    SearchLaneState() : fsm::State() {}

    void on_enter(fsm::Blackboard& bb) override {
        auto drone_ptr = bb.get<std::shared_ptr<Drone>>("drone");
        auto vision_ptr = bb.get<std::shared_ptr<VisionNode>>("vision");
        
        if (drone_ptr == nullptr || *drone_ptr == nullptr) {
            this->drone->log("ERROR: Drone pointer is null in SearchLaneState");
            return;
        }
        if (vision_ptr == nullptr || *vision_ptr == nullptr) {
            this->drone->log("ERROR: Vision pointer is null in SearchLaneState");
            return;
        }
        
        this->drone = *drone_ptr;
        this->vision = *vision_ptr;

        this->drone->log("STATE: SEARCH LANE STATE");

        this->max_velocity = *bb.get<float>("max_horizontal_velocity");
        this->drone->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f);
    }

    std::string act(fsm::Blackboard& bb) override {
        auto lane_data = this->vision->getCurrentLaneData();

        if(lane_data.area > 0){
            this->drone->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f);
            
            bool voltar = *bb.get<bool>("has_ever_detected_circle");
            if( ! (lane_data.is_circle && voltar))
                return "LANE FOUND";
        }
            
        Eigen::Vector3d local_velocity(0.1f, 0.0f, 0.0f);
        local_velocity = adjust_velocity_using_yaw(local_velocity, this->drone->getOrientation()[2]);
        this->drone->setLocalVelocity(local_velocity.x(), local_velocity.y(), local_velocity.z(), 0.0f);

        return "";
    }

    void on_exit(fsm::Blackboard& bb) override {
        (void) bb;
        this->drone->log("Exiting SearchLaneState");
    }
};