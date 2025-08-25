#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "vision_fase2.hpp"
#include "fase2/comandos.hpp"
#include "transformations.hpp"
#include "movement.hpp"

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
            this->drone->log("No marker detected, moving forward");

            move_local(this->drone, get_sentido(Direcoes::FRENTE), this->max_velocity);

            return "";
        }

        this->drone->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f); // parar movimento anterior
        
        this->drone->log("Marker detected: " + std::to_string(static_cast<int>(this->marker.dir)));
        
        return "MARKER DETECTED";
    }
};