#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "vision_fase2.hpp"

class ArucoTraverseState : public fsm::State {
private:
    std::shared_ptr<Drone> drone;

public:
    ArucoTraverseState() : fsm::State() {}



    void on_enter(fsm::Blackboard &bb) override {
        this->drone = *bb.get<std::shared_ptr<Drone>>("drone");
        
        if(this->drone == nullptr) return;

        this->drone->log("STATE: ARUCO TRAVERSE");
    }



    std::string act(fsm::Blackboard &bb) override {
        (void) bb;

        return "";
    }
};