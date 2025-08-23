#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <cmath>
#include <fstream>
#include <unistd.h>
#include <chrono>

class SoltarGarraState : public fsm::State {
public:
    SoltarGarraState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        
        if (drone == nullptr)
            return;
        
        drone->log("STATE: SOLTAR GARRA");

        start_time = std::chrono::steady_clock::now();
        i = 0;
    }

    std::string act(fsm::Blackboard &bb) override {
        (void)bb;
        
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
        
        if (i% 5 == 0){
            int ret = std::system(*bb.get<const char*>("command_to_drop_script"));
            drone->log("gancho_drop.py returned " + std::to_string(ret));
        }
        
        if (elapsed.count() > 5000) {  // 6 seconds total
            return "PACOTE ENTREGUE";
        }
        
        drone->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
        i++;
        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        drone->log("Exiting DROP GANCHO state - Claw opened successfully");
    }

private:
    Drone* drone;
    std::chrono::steady_clock::time_point start_time;
    int i;
};