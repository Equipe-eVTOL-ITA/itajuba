#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <cmath>
#include <fstream>
#include <unistd.h>
#include <chrono>

class SoltarGarraState : public fsm::State {
private:
    std::shared_ptr<Drone> drone;
    std::chrono::steady_clock::time_point start_time;
    int i;

public:
    SoltarGarraState() : fsm::State() {}

    void on_enter(fsm::Blackboard &bb) override {
        auto drone_ptr = bb.get<std::shared_ptr<Drone>>("drone");

        if(drone_ptr == nullptr){
            this->drone->log("ERROR: Drone pointer is null in SoltarGarraState");
            return;
        }

        this->drone = *drone_ptr;

        this->drone->log("STATE: SOLTAR GARRA");

        this->start_time = std::chrono::steady_clock::now();
        this->i = 0;
    }

    std::string act(fsm::Blackboard &bb) override {
        (void)bb;
        
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
        
        if (i% 5 == 0){
            std::string cmd = *bb.get<std::string>("command_to_drop");
            int ret = std::system(cmd.c_str());
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
};