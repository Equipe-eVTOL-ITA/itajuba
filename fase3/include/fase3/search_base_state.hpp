#include <Eigen/Eigen>
#include <iostream>
#include <vector>
#include "fsm/fsm.hpp"
#include "Base.hpp"
#include "ArenaPoint.hpp"
#include "drone/Drone.hpp"
#include "vision_fase3.hpp"

class SearchBaseState : public fsm::State {
public:
    SearchBaseState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        
        this->drone = *blackboard.get<std::shared_ptr<Drone>>("drone");
        this->vision = *blackboard.get<std::shared_ptr<VisionNode>>("vision");
        if(this->drone == nullptr || this->vision == nullptr) return;

        this->drone->log("");
        this->drone->log("STATE: SEARCH BASE");

        this->waypoints = blackboard.get<std::vector<ArenaPoint>>("waypoints");
        this->bases = blackboard.get<std::vector<Base>>("bases");
        this->target_shapes = blackboard.get<std::vector<std::string>>("target_shapes");

        this->print_counter = 0;

        this->position_tolerance = *blackboard.get<float>("position_tolerance");
        this->initial_yaw = 0.0f;
        this->max_velocity = *blackboard.get<float>("max_horizontal_velocity");
        this->known_base_radius = *blackboard.get<float>("known_base_radius");
        this->height_to_ground = *blackboard.get<float>("height_to_ground");
        this->mean_base_height = *blackboard.get<float>("mean_base_height");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        this->print_counter++;

        this->pos = this->drone->getLocalPosition();
        this->orientation = this->drone->getOrientation();

        if (this->vision->isThereDetection()) {
            auto bboxes = this->vision->getDetections();

            for (const auto& bbox : bboxes) {
                const Eigen::Vector3d approx_base = this->vision->getAccurateBase(this->pos, this->orientation, bbox);
                std::string detected_shape = bbox.class_id;  // Get the detected shape/color
                bool is_visited_shape = false;
                bool is_target_shape = false;
                
                // Check if this shape is in our target list
                if (this->target_shapes != nullptr) {
                    for (const std::string& target : *this->target_shapes) {
                        if (detected_shape == target) {
                            is_target_shape = true;
                            break;
                        }
                    }
                } else {
                    // If no target shapes specified, consider all shapes as targets
                    is_target_shape = true;
                }
                
                if (!is_target_shape) {
                    if (this->print_counter % 5 == 0) {
                        this->drone->log("Detected shape: " + detected_shape + " - not in target list, skipping!");
                    }
                    continue;  // Skip shapes not in target list
                }
                
                // Publish base detection to telemetry
                this->vision->publishBaseDetection("detected_base", approx_base); 
                
                if (this->print_counter % 5 == 0) {
                    this->drone->log("");
                    this->drone->log("Detected target shape: " + detected_shape + " at: {" + std::to_string(approx_base.x()) + ", " + std::to_string(approx_base.y()) + "}");
                }

                // Check if we've already landed on this shape/color
                for (const auto& base : *this->bases) {
                    if (base.shape_class == detected_shape && base.is_visited) {
                        if (this->print_counter % 5 == 0) {
                            this->drone->log("Already landed on shape: " + detected_shape + " - skipping!");
                        }
                        is_visited_shape = true;
                        break;
                    }
                }

                // Only proceed if we haven't landed on this shape before
                if (!is_visited_shape) {
                    this->drone->log("");
                    this->drone->log("New target shape: " + detected_shape + " at: {" + std::to_string(approx_base.x()) + ", " + std::to_string(approx_base.y()) + "}");
                    
                    // Publish first detection as yellow marker
                    this->vision->publishBaseDetection("first_estimate_base", approx_base); 
                    
                    blackboard.set<Eigen::Vector3d>("approximate_base", approx_base);
                    blackboard.set<std::string>("target_shape", detected_shape);  // Store the target shape
                    return "BASE FOUND";
                }
            }
        } 

        // Check if we've completed all target shapes
        if (this->target_shapes != nullptr && this->bases != nullptr) {
            int completed_targets = 0;
            for (const std::string& target : *this->target_shapes) {
                for (const auto& base : *this->bases) {
                    if (base.shape_class == target && base.is_visited) {
                        completed_targets++;
                        break;  // Found this target, move to next
                    }
                }
            }
            
            if (completed_targets >= static_cast<int>(this->target_shapes->size())) {
                this->drone->log("");
                this->drone->log("All target shapes completed! Mission finished.");
                return "SEARCH ENDED";
            }
        }

        auto goal_point = this->getNextPoint(this->waypoints);
        if(goal_point == nullptr) return "SEARCH ENDED";
        
        Eigen::Vector3d goal = goal_point->coordinates;
        Eigen::Vector3d goal_diff = goal - this->pos;

        if (goal_diff.norm() < this->position_tolerance) {
            goal_point->is_visited = true;
            this->drone->log("");
            this->drone->log("Ponto visitado: {" + std::to_string(pos.x()) + ", "
                            + std::to_string(pos.y()) + ", " + std::to_string(pos.z()) + "}");
            return ""; 
        }

        Eigen::Vector3d little_goal = this->pos + (goal_diff.norm() > max_velocity ? goal_diff.normalized() * max_velocity : goal_diff);

        this->drone->setLocalPosition(
            little_goal[0],
            little_goal[1],
            little_goal[2],
            this->initial_yaw
        );

        return "";

    }

    void on_exit(fsm::Blackboard &blackboard) override {
        this->pos = this->drone->getLocalPosition();
        blackboard.set<Eigen::Vector3d>("last search position", this->pos);
    }

private:
    std::shared_ptr<Drone> drone;
    std::shared_ptr<VisionNode> vision;
    std::vector<ArenaPoint>* waypoints;
    std::vector<Base>* bases;
    std::vector<std::string>* target_shapes;

    float initial_yaw;
    float position_tolerance;
    float max_velocity;

    int print_counter;

    float known_base_radius;
    float height_to_ground;
    float mean_base_height;

    Eigen::Vector3d pos, orientation;
    
    ArenaPoint* getNextPoint(std::vector<ArenaPoint>* waypoints) {
        if (!waypoints) return nullptr;
        for (auto& point : *waypoints) {
            if (!point.is_visited) {
                return &point;
            }
        }
        // Return nullptr if no unvisited points are found
        return nullptr;
    }
};