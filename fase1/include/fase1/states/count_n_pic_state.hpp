#pragma once

#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "fase1/vision_fase1.hpp"
#include "fase1/bases.hpp"
#include <chrono>

class CountNPicState : public fsm::State {
private:
    std::shared_ptr<Drone> drone;
    std::shared_ptr<VisionNode> vision;
    
    Base base;

    int counter_bases;
    int max_bases;
    
public:
    CountNPicState() : fsm::State() {}

    void on_enter(fsm::Blackboard &bb) override {
        auto drone_ptr = bb.get<std::shared_ptr<Drone>>("drone");
        auto vision_ptr = bb.get<std::shared_ptr<VisionNode>>("vision");

        if(drone_ptr == nullptr || vision_ptr == nullptr) return;

        this->drone = *drone_ptr;
        this->vision = *vision_ptr;

        this->drone->log("STATE: COUNT N PIC");

        this->counter_bases = 0;
        this->max_bases = *bb.get<int>("max_number_of_bases");
    }

    std::string act(fsm::Blackboard &bb) override {
        (void)bb;

        auto base = this->vision->getCurrentBase();

        if(base.forma != NENHUMA_FORMA) {
            this->counter_bases++;

            std::string forma_nome = forma_map[base.forma];
            this->drone->log("Base detectada: " + forma_nome + " (Total: " + std::to_string(this->counter_bases) + ")");

            this->vision->publishCounter(this->counter_bases);

            return (this->counter_bases >= this->max_bases) ? "ALL BASES FOUND" : "NEXT WAYPOINT";
        }

        return "";
    }
};