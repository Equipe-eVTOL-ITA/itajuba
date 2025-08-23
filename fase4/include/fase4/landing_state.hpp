#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <chrono>

class LandingState : public fsm::State {
private:
    std::shared_ptr<Drone> drone;

    std::chrono::steady_clock::time_point start_time_;

    float landing_velocity;
    float landing_timeout;

public:
    LandingState() {}

    void on_enter(fsm::Blackboard &bb) override {
        auto drone_ptr = bb.get<std::shared_ptr<Drone>>("drone");

        if(drone_ptr == nullptr){
            this->drone->log("ERROR: Drone pointer is null in LandingState");
            return;
        }

        this->drone = *drone_ptr;

        this->drone->log("STATE: LANDING STATE");

        this->landing_velocity = *bb.get<float>("landing_velocity");
        this->landing_timeout = *bb.get<float>("landing_timeout");

        this->start_time_ = std::chrono::steady_clock::now();
    }

    std::string act(fsm::Blackboard &bb) override {
        (void) bb;

        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time-this->start_time_).count();

        if(elapsed_time > this->landing_timeout){
            if(*bb.get<bool>("has_ever_landed") == false){
                bb.set<bool>("has_ever_landed", true);
                return "LANDED FOR THE FIRST TIME";
            }
            return "LANDED";
        }

        this->drone->setLocalVelocity(0.0, 0.0, this->landing_velocity, 0.0);

        return "";
    }

};