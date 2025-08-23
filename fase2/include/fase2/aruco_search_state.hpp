#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "vision_fase2.hpp"
#include "fase2/comandos.hpp"
#include "transformations.hpp"

class ArucoSearchState : public fsm::State {
private:
    std::shared_ptr<Drone> drone;
    std::shared_ptr<VisionNode> vision;

    ArucoMarker marker;

    float max_velocity;
    float height;

public:
    ArucoSearchState() : fsm::State() {}



    void on_enter(fsm::Blackboard &bb) override {
        auto drone_ptr = bb.get<std::shared_ptr<Drone>>("drone");
        auto vision_ptr = bb.get<std::shared_ptr<VisionNode>>("vision");

        if(drone_ptr == nullptr || vision_ptr == nullptr) return;

        this->drone = *drone_ptr;
        this->vision = *vision_ptr;

        this->drone->log("STATE: ARUCO SEARCH");

        this->max_velocity = *bb.get<float>("max_horizontal_velocity");
    }



    std::string act(fsm::Blackboard &bb) override {
        (void) bb;

        this->marker = this->vision->getCurrentMarker();
        if(this->marker.dir == Direcoes::NENHUMA){
            Eigen::Vector3d local_velocity = adjust_velocity_using_yaw(DIRECTIONS.at(Direcoes::FRENTE), this->drone->getOrientation()[2]);
            this->drone->setLocalVelocity(local_velocity.x(), local_velocity.y(), local_velocity.z(), 0.0f);
            return "";
        }
        
        return "MARKER DETECTED";
    }
};