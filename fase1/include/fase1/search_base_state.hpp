#include <Eigen/Eigen>
#include <iostream>
#include <vector>
#include <limits>
#include <map>
#include <string>
#include "fsm/fsm.hpp"
#include "Base.hpp"
#include "ArenaPoint.hpp"
#include "drone/Drone.hpp"
#include "vision_fase1.hpp"

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
        this->detection_counters.clear();
        this->detection_positions.clear();
        this->detection_shapes.clear();

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
            
            // Track current detections in this frame
            std::vector<std::string> current_frame_detections;

            for (const auto& bbox : bboxes) {
                const Eigen::Vector3d approx_base = this->vision->getApproximateBase(this->pos, this->orientation, bbox, this->mean_base_height);
                std::string detected_shape = bbox.class_id;  // Get the detected shape/color
                bool is_known_base = false;
                bool is_target_shape = false;
                float min_horizontal_distance = std::numeric_limits<float>::max();
                
                // Publish base detection to telemetry
                this->vision->publishBaseDetection("detected_base", approx_base); 
                
                if (this->print_counter % 5 == 0) {
                    this->drone->log("");
                    this->drone->log("Estimate: {" + std::to_string(approx_base.x()) + ", " + std::to_string(approx_base.y()) + "}");
                }
                
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

                // Check if detection is within radius of any known base
                for (const auto& base : *this->bases) {
                    float horizontal_distance = (approx_base.head<2>() - base.coordinates.head<2>()).norm();

                    min_horizontal_distance = std::min(min_horizontal_distance, horizontal_distance);
                    
                    if (this->print_counter % 5 == 0) {
                        std::string base_index = std::to_string(&base - &(*this->bases)[0]);
                        this->drone->log("Dist " + std::to_string(horizontal_distance) + " to Base {" + base_index + "}: {"
                                    + std::to_string(base.coordinates[0]) + ", " + std::to_string(base.coordinates[1]) + "}");
                    }
                    
                    if (horizontal_distance < this->known_base_radius) {
                        if (this->print_counter % 5 == 0) {
                            this->drone->log("Known base within radius!");
                        }
                        is_known_base = true;
                        break;
                    }
                }

                // Only proceed if this is NOT a known base (new base detected)
                if (!is_known_base) {
                    // Create unique key for this detection (position + shape)
                    Eigen::Vector2d current_detection_pos = approx_base.head<2>();
                    std::string detection_key = this->createDetectionKey(current_detection_pos, detected_shape);
                    
                    // Add to current frame detections
                    current_frame_detections.push_back(detection_key);
                    
                    // Check if this detection already exists (within 2m radius)
                    std::string existing_key = this->findExistingDetection(current_detection_pos, detected_shape);
                    
                    if (!existing_key.empty()) {
                        // Update existing detection
                        this->detection_counters[existing_key]++;
                        if (this->print_counter % 10 == 0) {
                            this->drone->log("Updating detection: " + detected_shape + 
                                           " (" + std::to_string(this->detection_counters[existing_key]) + "/50 detections)");
                        }
                        
                        // Check if this detection reached 50 counts
                        if (this->detection_counters[existing_key] >= 50) {
                            this->drone->log("");
                            this->drone->log("BASE CONFIRMED after 50 detections!");
                            this->drone->log("New base at: {" + std::to_string(approx_base.x()) + ", " + 
                                           std::to_string(approx_base.y()) + "}");
                            this->drone->log("Target shape: " + detected_shape);
                            this->drone->log("Min distance to known base: " + std::to_string(min_horizontal_distance));
                            
                            // Publish first detection as yellow marker
                            this->vision->publishBaseDetection("first_estimate_base", approx_base); 
                            
                            blackboard.set<Eigen::Vector3d>("approximate_base", approx_base);
                            blackboard.set<std::string>("target_shape", detected_shape);  // Store the target shape
                            return "BASE FOUND";
                        }
                    } else {
                        // New detection
                        this->detection_counters[detection_key] = 1;
                        this->detection_positions[detection_key] = current_detection_pos;
                        this->detection_shapes[detection_key] = detected_shape;
                        this->drone->log("New base detected at: {" + std::to_string(approx_base.x()) + ", " + 
                                       std::to_string(approx_base.y()) + "} - starting counter (1/50)");
                        this->drone->log("Shape: " + detected_shape);
                        this->drone->log("Min distance to known base: " + std::to_string(min_horizontal_distance));
                    }
                }
            }
            
            // Clean up detections that weren't seen in this frame (reset their counters)
            this->cleanupMissedDetections(current_frame_detections);
        } else {
            // Reset all counters if no detection
            if (!this->detection_counters.empty()) {
                if (this->print_counter % 10 == 0) {
                    this->drone->log("No detection - resetting all counters");
                }
                this->detection_counters.clear();
                this->detection_positions.clear();
                this->detection_shapes.clear();
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
    
    // Multiple detection tracking
    std::map<std::string, int> detection_counters;          // detection_key -> count
    std::map<std::string, Eigen::Vector2d> detection_positions;  // detection_key -> position
    std::map<std::string, std::string> detection_shapes;    // detection_key -> shape

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
    
    // Helper functions for multiple detection tracking
    std::string createDetectionKey(const Eigen::Vector2d& position, const std::string& shape) {
        return std::to_string(static_cast<int>(position.x() * 10)) + "_" + 
               std::to_string(static_cast<int>(position.y() * 10)) + "_" + shape;
    }
    
    std::string findExistingDetection(const Eigen::Vector2d& position, const std::string& shape) {
        for (const auto& pair : this->detection_positions) {
            if (this->detection_shapes[pair.first] == shape &&
                (pair.second - position).norm() < 2.0) {  // Within 2m radius
                return pair.first;
            }
        }
        return "";  // Not found
    }
    
    void cleanupMissedDetections(const std::vector<std::string>& current_detections) {
        // Remove detections that weren't seen in this frame
        auto it = this->detection_counters.begin();
        while (it != this->detection_counters.end()) {
            bool found_in_current_frame = false;
            for (const auto& current_key : current_detections) {
                if (this->findExistingDetection(this->detection_positions[it->first], 
                                              this->detection_shapes[it->first]) == it->first) {
                    found_in_current_frame = true;
                    break;
                }
            }
            
            if (!found_in_current_frame) {
                if (this->print_counter % 15 == 0) {
                    this->drone->log("Removing missed detection: " + this->detection_shapes[it->first]);
                }
                this->detection_positions.erase(it->first);
                this->detection_shapes.erase(it->first);
                it = this->detection_counters.erase(it);
            } else {
                ++it;
            }
        }
    }
};