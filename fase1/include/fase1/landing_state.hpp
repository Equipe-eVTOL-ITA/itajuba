#include <Eigen/Eigen>
#include <chrono>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "vision_fase1.hpp"
#include "Base.hpp"


class LandingState : public fsm::State {
public:
    LandingState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        this->drone = *blackboard.get<std::shared_ptr<Drone>>("drone");
        this->vision = *blackboard.get<std::shared_ptr<VisionNode>>("vision");
        if (this->vision == nullptr || this->drone == nullptr) return;

        this->drone->log("");
        this->drone->log("STATE: SIMULATED LANDING (Fase1 - No actual landing)");

        // For fase1, we just simulate a quick landing without actually descending
        this->timeout_ = 2.0;  // Quick 2-second simulation
        this->start_time_ = std::chrono::steady_clock::now();
        
        // Store the position and orientation when entering the state
        this->target_position_ = this->drone->getLocalPosition();
        this->target_orientation_ = this->drone->getOrientation();

        this->drone->log("Simulating landing for 2 seconds...");
        this->drone->log("Maintaining position: " + std::to_string(this->target_position_.x()) + 
                        ", " + std::to_string(this->target_position_.y()) + 
                        ", " + std::to_string(this->target_position_.z()));
    }
    std::string act(fsm::Blackboard &blackboard) override {
        (void) blackboard;
        
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - this->start_time_).count();

        // For fase1, maintain the fixed position from when we entered the state
        this->drone->setLocalPosition(this->target_position_.x(), 
                                     this->target_position_.y(), 
                                     this->target_position_.z(), 
                                     this->target_orientation_.z());

        if (elapsed_time > this->timeout_){
            return "LANDED";
        }

        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        Eigen::Vector3d pos = this->drone->getLocalPosition();
        Eigen::Vector3d gps_pos = this->drone->getGlobalPosition();
        
        this->vision->publishBaseDetection("confirmed_base", pos);

        auto bases = blackboard.get<std::vector<Base>>("bases");
        
        // Get the target shape that was detected (default to "unknown")
        std::string landed_shape = "unknown";
        auto target_shape_ptr = blackboard.get<std::string>("target_shape");
        if (target_shape_ptr != nullptr) {
            landed_shape = *target_shape_ptr;
        }
        
        // Create a new base with shape information and mark as visited
        Base new_base;
        new_base.coordinates = pos;
        new_base.shape_class = landed_shape;
        new_base.is_visited = true;
        
        bases->push_back(new_base);

        // Log both local and GPS coordinates
        drone->log("Landed on shape: " + landed_shape + " at base {" + std::to_string(bases->size()) + "}:");
        drone->log("  Local position (FRD): " + std::to_string(pos.x()) + ", " + 
                  std::to_string(pos.y()) + ", " + std::to_string(pos.z()));
        drone->log("  GPS coordinates: Lat=" + std::to_string(gps_pos.x()) + "°, " +
                  "Lon=" + std::to_string(gps_pos.y()) + "°, " +
                  "Alt=" + std::to_string(gps_pos.z()) + "m AMSL");

        auto num_bases = *blackboard.get<float>("num_bases");
        size_t total_bases = static_cast<size_t>(num_bases);

        if (bases->size() == total_bases + 1){  // +1 because home is also in the vector
            blackboard.set<bool>("finished_bases", true);
            drone->log("Visited all " + std::to_string(total_bases) + " bases");
        }
    }

private:
    std::shared_ptr<Drone> drone;
    std::shared_ptr<VisionNode> vision;

    float v_max, v_min;
    float time_constant;
    float timeout_;
    std::chrono::steady_clock::time_point start_time_;
    
    // Store the target position and orientation to maintain
    Eigen::Vector3d target_position_;
    Eigen::Vector3d target_orientation_;
};