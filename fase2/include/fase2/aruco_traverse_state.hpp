#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "vision_fase2.hpp"
#include "fase2/comandos.hpp"

class ArucoTraverseState : public fsm::State {
private:
    std::shared_ptr<Drone> drone;
    std::shared_ptr<VisionNode> vision;

    ArucoMarker marker;
    ArucoMarker first_marker;

    float max_velocity;
    float height;

public:
    ArucoTraverseState() : fsm::State() {}



    void on_enter(fsm::Blackboard &bb) override {
        auto drone_ptr = bb.get<std::shared_ptr<Drone>>("drone");
        auto vision_ptr = bb.get<std::shared_ptr<VisionNode>>("vision");

        if(drone_ptr == nullptr || vision_ptr == nullptr) return;

        this->drone = *drone_ptr;
        this->vision = *vision_ptr;

        this->drone->log("STATE: ARUCO TRAVERSE");

        this->max_velocity = *bb.get<float>("max_horizontal_velocity");
        this->height = *bb.get<float>("takeoff_height");

        this->first_marker = this->vision->getCurrentMarker();

    }



    std::string act(fsm::Blackboard &bb) override {
        (void) bb;

        auto marker = this->vision->getCurrentMarker();

        if (marker.dir == Direcoes::NENHUMA)
            return "NO MARKER";

        auto local_velocity = DIRECTIONS.at(marker.dir) * this->max_velocity;
        this->drone->setLocalVelocity(local_velocity.x(), local_velocity.y(), local_velocity.z(), 0.0f);

        if(marker.dir != this->first_marker.dir)
            return "DIFFERENT MARKER";

        return "";
    }
};