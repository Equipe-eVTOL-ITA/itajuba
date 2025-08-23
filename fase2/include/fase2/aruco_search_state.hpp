#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "vision_fase2.hpp"
#include "fase2/comandos.hpp"

class ArucoSearchState : public fsm::State {
private:
    std::shared_ptr<Drone> drone;
    std::shared_ptr<VisionNode> vision;

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
        return "";
    }
};