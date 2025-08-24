#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "vision_fase2.hpp"
#include "fase2/comandos.hpp"
#include "fase2/movement.hpp"

class ArucoTraverseState : public fsm::State
{
private:
    std::shared_ptr<Drone> drone;
    std::shared_ptr<VisionNode> vision;

    ArucoMarker marker;
    ArucoMarker first_marker;

    Direcoes last_direction = Direcoes::NENHUMA;

    float max_velocity;
    float height;

public:
    static_assert(std::is_enum<Direcoes>::value, "Direcoes must be an enum");
    ArucoTraverseState() : fsm::State() {}

    void on_enter(fsm::Blackboard &bb) override
    {
        auto drone_ptr = bb.get<std::shared_ptr<Drone>>("drone");
        auto vision_ptr = bb.get<std::shared_ptr<VisionNode>>("vision");

        if (drone_ptr == nullptr || vision_ptr == nullptr)
            return;

        this->drone = *drone_ptr;
        this->vision = *vision_ptr;

        this->drone->log("STATE: ARUCO TRAVERSE");

        this->max_velocity = *bb.get<float>("max_horizontal_velocity");
        this->height = *bb.get<float>("takeoff_height");

        this->first_marker = this->vision->getCurrentMarker();
    }

    std::string act(fsm::Blackboard &bb) override
    {
        (void)bb;

        auto marker = this->vision->getCurrentMarker();

        if(marker.dir == Direcoes::NENHUMA) {
            this->drone->log("No marker detected, just believing... last_direction=" + std::to_string(static_cast<int>(this->last_direction)));
            Direcoes dir_key = static_cast<Direcoes>(this->last_direction);
            this->drone->log("[DEBUG] Attempting get_direction_vector(last_direction) with value: " + std::to_string(static_cast<int>(dir_key)));
            move_local(this->drone, get_direction_vector(dir_key), this->max_velocity);
            return "";
        }

        if (marker.dir == Direcoes::POUSAR)
            return "POUSAR";

        this->drone->log("Moving towards marker " + std::to_string(static_cast<int>(marker.dir)));
        Direcoes current_dir = static_cast<Direcoes>(marker.dir);
        if(this->last_direction != marker.dir) {
            this->drone->log("[DEBUG] Updating last_direction from " + std::to_string(static_cast<int>(this->last_direction)) + " to " + std::to_string(static_cast<int>(marker.dir)));
            this->last_direction = marker.dir;
        }
        this->drone->log("[DEBUG] Attempting get_direction_vector(current_dir) with value: " + std::to_string(static_cast<int>(current_dir)));
        move_local(this->drone, get_direction_vector(current_dir), this->max_velocity);
        if (marker.dir != this->first_marker.dir)
            return "DIFFERENT MARKER";
        return "";
    }
};