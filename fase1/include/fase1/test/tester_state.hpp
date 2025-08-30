#include <Eigen/Eigen>
#include <iostream>
#include <vector>
#include "fsm/fsm.hpp"
#include "../Base.hpp"
#include "../ArenaPoint.hpp"
#include "drone/Drone.hpp"
#include "../vision_fase1.hpp"


class TesterState : public fsm::State {
public:
    TesterState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        
        this->drone = *blackboard.get<std::shared_ptr<Drone>>("drone");
        this->vision = *blackboard.get<std::shared_ptr<VisionNode>>("vision");
        if(this->drone == nullptr || this->vision == nullptr) return;

        this->drone->log("");
        this->drone->log("STATE: TESTER");

        float home_x = *blackboard.get<float>("fictual_home_x");
        float home_y = *blackboard.get<float>("fictual_home_y");
        float home_z = *blackboard.get<float>("fictual_home_z");
        const Eigen::Vector3d fictual_home = Eigen::Vector3d({home_x, home_y, home_z});
        blackboard.set<Eigen::Vector3d>("home_position", fictual_home);

        this->drone->toOffboardSync();
        this->drone->setHomePosition(fictual_home);
        
        this->pos = this->drone->getLocalPosition();

        // Initialize with detected base at current position for testing
        Base base{this->pos, "test_shape", false};
        this->vision->publishBaseDetection("confirmed_base", this->pos);
        
        std::vector<Base> bases_vector;
        bases_vector.push_back(base);
        blackboard.set<std::vector<Base>>("bases", bases_vector);

        this->waypoints = blackboard.get<std::vector<ArenaPoint>>("waypoints");
        this->bases = blackboard.get<std::vector<Base>>("bases");
        this->target_shapes = blackboard.get<std::vector<std::string>>("target_shapes");

        this->print_counter = 0;

        this->known_base_radius = *blackboard.get<float>("known_base_radius");
        this->height_to_ground = *blackboard.get<float>("height_to_ground");
        this->mean_base_height = *blackboard.get<float>("mean_base_height");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        this->print_counter++;

        this->pos = this->drone->getLocalPosition();
        this->orientation = this->drone->getOrientation();

        if (this->vision->isThereDetection()) {
            auto bboxes = this->vision->getDetections();

            for (const auto& bbox : bboxes) {
                const Eigen::Vector3d approx_base = this->vision->getApproximateBase(this->pos, this->orientation, bbox, this->mean_base_height);
                bool is_known_base = false;
                float min_horizontal_distance = std::numeric_limits<float>::max();
                
                // Publish base detection to telemetry
                this->vision->publishBaseDetection("detected_base", approx_base); 
                
                if (this->print_counter % 5 == 0) {
                    this->drone->log("");
                    this->drone->log("Detected shape: " + bbox.class_id);
                    this->drone->log("Estimate: {" + std::to_string(approx_base.x()) + ", " + std::to_string(approx_base.y()) + "}");
                    this->drone->log("Confidence: " + std::to_string(bbox.confidence));
                }

                // Check if this shape is in our target list
                bool is_target_shape = false;
                if (target_shapes) {
                    for (const auto& target : *target_shapes) {
                        if (bbox.class_id == target) {
                            is_target_shape = true;
                            break;
                        }
                    }
                }

                for (const auto& base : *this->bases) {
                    float horizontal_distance = (approx_base.head<2>() - base.coordinates.head<2>()).norm();

                    min_horizontal_distance = std::min(min_horizontal_distance, horizontal_distance);
                    
                    if (horizontal_distance < this->known_base_radius) {
                        if (this->print_counter % 5 == 0) {
                            std::string base_index = std::to_string(&base - &(*this->bases)[0]);
                            this->drone->log("Dist " + std::to_string(horizontal_distance) + " to Base {" + base_index + "}: {"
                                        + std::to_string(base.coordinates[0]) + ", " + std::to_string(base.coordinates[1]) + "}");
                            this->drone->log("Shape: " + base.shape_class);
                            this->drone->log("Known base!");
                        }
                        is_known_base = true;
                        break;
                    }
                }

                if (!is_known_base && is_target_shape) {
                    this->drone->log("");
                    this->drone->log("New TARGET base at: {" + std::to_string(approx_base.x()) + ", " + std::to_string(approx_base.y()) + "}");
                    this->drone->log("Shape: " + bbox.class_id);
                    this->drone->log("Min distance to known base: " + std::to_string(min_horizontal_distance));
                    
                    // Publish confirmed detection
                    this->vision->publishBaseDetection("confirmed_base", approx_base, bbox.confidence); 

                    bases->push_back({Eigen::Vector3d({approx_base.x(), approx_base.y(), approx_base.z()}), bbox.class_id, false});
                    blackboard.set<std::vector<Base>>("bases", *this->bases);
                    
                    return "";
                } else if (!is_known_base && !is_target_shape) {
                    if (this->print_counter % 10 == 0) {
                        this->drone->log("Detected non-target shape: " + bbox.class_id + " at {" + 
                                       std::to_string(approx_base.x()) + ", " + std::to_string(approx_base.y()) + "}");
                    }
                }
            }
        } 

        // Log current status periodically
        if (this->print_counter % 20 == 0) {
            this->drone->log("");
            this->drone->log("=== TESTER STATUS ===");
            this->drone->log("Current position: {" + std::to_string(this->pos.x()) + ", " + 
                           std::to_string(this->pos.y()) + ", " + std::to_string(this->pos.z()) + "}");
            this->drone->log("Known bases: " + std::to_string(this->bases->size()));
            
            if (target_shapes) {
                this->drone->log("Target shapes: ");
                for (const auto& shape : *target_shapes) {
                    this->drone->log("  - " + shape);
                }
            }
            this->drone->log("====================");
        }

        return "";
    }

private:
    std::shared_ptr<Drone> drone;
    std::shared_ptr<VisionNode> vision;
    std::vector<ArenaPoint>* waypoints;
    std::vector<Base>* bases;
    std::vector<std::string>* target_shapes;

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
