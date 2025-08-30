#include <memory>
#include <iostream>
#include <vector>

#include "fsm/fsm.hpp"
#include <rclcpp/rclcpp.hpp>
#include "drone/Drone.hpp"

#include "fase1/vision_fase1.hpp"
#include "fase1/test/tester_state.hpp"


class Fase1TesterFSM : public fsm::FSM {
public:
    Fase1TesterFSM(
        std::shared_ptr<Drone> drone,
        std::shared_ptr<VisionNode> vision,
        const std::map<std::string, std::variant<double, std::string>>& params,
        const std::vector<std::string>& target_shapes
    ) : fsm::FSM({"ERROR", "FINISHED"}) {
        
        this->blackboard_set<std::shared_ptr<Drone>>("drone", drone);
        this->blackboard_set<std::shared_ptr<VisionNode>>("vision", vision);
        this->blackboard_set<std::vector<std::string>>("target_shapes", target_shapes);

        // Parametros de ROS 2
        for (const auto& [key, value] : params) {
            if (std::holds_alternative<double>(value)) {
                this->blackboard_set<float>(key, static_cast<float>(std::get<double>(value)));
            } else if (std::holds_alternative<std::string>(value)) {
                this->blackboard_set<std::string>(key, std::get<std::string>(value));
            }
        }
        
        // DYNAMIC GRID - Similar to fase1.cpp but adapted for testing -------

        float takeoff_height = *this->blackboard_get<float>("takeoff_height");
        float home_x = *this->blackboard_get<float>("fictual_home_x");
        float home_y = *this->blackboard_get<float>("fictual_home_y");
        float step_size = *this->blackboard_get<float>("grid_step_x"); // Using this as spiral step size

        // Create a spiral square with 9 points (1 loop)
        std::vector<ArenaPoint> waypoints;

        // Starting point (center)
        waypoints.push_back({Eigen::Vector3d({home_x, home_y, takeoff_height})});

        // Direction vectors for the spiral: right, up, left, down
        const std::vector<std::pair<int, int>> directions = {
            {1, 0},  // right
            {0, 1},  // up
            {-1, 0}, // left
            {0, -1}  // down
        };

        int x = 0;
        int y = 0;
        int dir = 0;        // Start by going right
        int segment_length = 1; // Initial segment length
        int steps_taken = 0;

        // We need 8 more points after the center (total of 9)
        for (int i = 0; i < 8; i++) {
            // Move in current direction
            x += directions[dir].first;
            y += directions[dir].second;
            
            // Add waypoint
            float x_coord = home_x + x * step_size;
            float y_coord = home_y + y * step_size;
            waypoints.push_back({Eigen::Vector3d({x_coord, y_coord, takeoff_height})});
            
            // Track steps taken in current segment
            steps_taken++;
            
            // Check if we need to change direction
            if (steps_taken == segment_length) {
                dir = (dir + 1) % 4; // Change direction (right->up->left->down)
                steps_taken = 0;     // Reset steps counter
                
                // Increase segment length after completing a half-loop (two segments)
                if (dir % 2 == 0) {
                    segment_length++;
                }
            }
        }

        this->blackboard_set<std::vector<ArenaPoint>>("waypoints", waypoints);
        this->blackboard_set<bool>("finished_bases", false);

        // ------------------------------------------------------------------


        // STATE MACHINE ----------------------------------------------------


        this->add_state("TESTER", std::make_unique<TesterState>());

        this->add_transitions("TESTER", {
            {"TEST COMPLETE", "FINISHED"},
            {"SEG FAULT", "ERROR"}
        });

        // Set initial state
        this->set_initial_state("TESTER");
    }

};


class NodeFSM : public rclcpp::Node {
public:
    NodeFSM(std::shared_ptr<Drone> drone, std::shared_ptr<VisionNode> vision) 
        : rclcpp::Node("fase1_fsm"), drone_node_(drone), vision_node_(vision) {


        std::map<std::string, std::variant<double, std::string>> default_params = {
            {"fictual_home_x", 0.0},
            {"fictual_home_y", 0.0},
            {"fictual_home_z", 0.0},

            {"num_bases", 3.0},
            {"known_base_radius", 1.7},
            {"height_to_ground", 1.1},
            
            {"grid_y_length", -3.3},
            {"grid_step_x", 3.3},
            {"grid_num_steps", 3.0},

            {"takeoff_height", -5.0},
            {"align_height", -4.0},
            {"max_base_height", 0.0},
            {"mean_base_height", 0.0},

            {"max_vertical_velocity", 1.2},
            {"max_horizontal_velocity", 1.0},
            {"landing_velocity_max", 0.7},
            {"landing_velocity_min", 0.4},
            {"align_descent_velocity", 0.15},

            {"max_search_time", 30.0},
            {"detection_timeout", 10.0},

            {"position_tolerance", 0.05},
            {"align_tolerance", 0.025},

            {"pid_pos_kp", 1.0},
            {"pid_pos_ki", 0.01},
            {"pid_pos_kd", 0.05},
            {"setpoint", 0.0}
        };
        
        auto params = declareAndGetParameters(default_params);

        // Handle target_shapes parameter separately (string array)
        this->declare_parameter<std::vector<std::string>>("target_shapes", std::vector<std::string>{"cruz", "triangulo", "quadrado"});
        std::vector<std::string> target_shapes = this->get_parameter("target_shapes").as_string_array();

        fsm_ = std::make_unique<Fase1TesterFSM>(drone_node_, vision_node_, params, target_shapes);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&NodeFSM::executeFSM, this)
        );

    }

    void executeFSM() {
        if (rclcpp::ok() && !fsm_->is_finished()) {
            fsm_->execute();
        } else {
            rclcpp::shutdown();
        }
    }

private:
    std::shared_ptr<Drone> drone_node_;
    std::shared_ptr<VisionNode> vision_node_;
    std::unique_ptr<Fase1TesterFSM> fsm_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::map<std::string, std::variant<double, std::string>> declareAndGetParameters(
        const std::map<std::string, std::variant<double, std::string>>& defaults) {
        
        std::map<std::string, std::variant<double, std::string>> result;
        
        for (const auto& [name, default_value] : defaults) {
            if (std::holds_alternative<double>(default_value)) {
                this->declare_parameter(name, std::get<double>(default_value));
                result[name] = this->get_parameter(name).as_double();
            } else if (std::holds_alternative<std::string>(default_value)) {
                this->declare_parameter(name, std::get<std::string>(default_value));
                result[name] = this->get_parameter(name).as_string();
            }
        }
        
        return result;
    }
};


int main(int argc, const char *argv[]){
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;
    
    auto drone = std::make_shared<Drone>();
    auto vision = std::make_shared<VisionNode>();
    auto fsm_node = std::make_shared<NodeFSM>(drone, vision);
    
    executor.add_node(drone);
    executor.add_node(vision);
    executor.add_node(fsm_node);

    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
