#include <memory>
#include <iostream>
#include <vector>

#include "fsm/fsm.hpp"
#include <rclcpp/rclcpp.hpp>
#include "drone/Drone.hpp"
#include "vision_fase4.hpp"

// Include all state headers
#include "fase4/arming_state.hpp"
#include "fase4/takeoff_state.hpp"

class Fase4FSM : public fsm::FSM {
public:
    Fase4FSM(std::shared_ptr<Drone> drone, std::shared_ptr<VisionNode> vision) : fsm::FSM({"ERROR", "FINISHED"}){
        this->blackboard_set<std::shared_ptr<Drone>>("drone", drone);
        this->blackboard_set<std::shared_ptr<VisionNode>>("vision", vision);

        // Máquina de Estados
        this->add_state("ARMING", std::make_unique<ArmingState>());
        this->add_state("INITIAL TAKEOFF", std::make_unique<InitialTakeoffState>());
        
        this->set_initial_state("ARMING");

        // Transições de Estados
        this->add_transition("ARMING", {
            {"ARMED", "INITIAL TAKEOFF"},
            {"NOT ARMED", "ERROR"},
            {"SEG FAULT", "ERROR"}
        });

    }
};

class NodeFSM : public rclcpp::Node {
private:
    std::shared_ptr<Drone> drone_node_;
    std::shared_ptr<VisionNode> vision_node_;
    std::unique_ptr<Fase4FSM> fsm_;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    NodeFSM(std::shared_ptr<Drone> drone, std::shared_ptr<VisionNode> vision) : rclcpp::Node("fase4_fsm"), drone_node_(drone), vision_node_(vision) {
        
        std::map<std::string, std::variant<double, std::string>> defaults = {
            {"fictual_home_x", 0.0},
            {"fictual_home_y", 0.0},
            {"fictual_home_z", 0.0},
            {"max_vertical_velocity", 0.5},
            {"max_horizontal_velocity", 1.0},
            {"position_tolerance", 0.1},
            {"takeoff_height", 1.5},
            {"pid_pos_kp", 0.5},
            {"pid_pos_ki", 0.0},
            {"pid_pos_kd", 0.1},
            {"setpoint", 0.0}
        };

        fsm_ = std::make_unique<Fase4FSM>(drone_node_, vision_node_);
        
        auto parameters = declareAndGetParameters(
            defaults,
            // Lambda para lidar com parâmetros do tipo double - captura this
            [this](const std::string& name, double default_value) -> double {
                this->declare_parameter(name, default_value);
                return this->get_parameter(name).as_double();
            },
            // Lambda para lidar com parâmetros do tipo string - captura this
            [this](const std::string& name, const std::string& default_value) -> std::string {
                this->declare_parameter(name, default_value);
                return this->get_parameter(name).as_string();
            }
        );
        
        // Isso permite fazer:
        // double takeoff_height = std::get<double>(parameters["takeoff_height"]);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&NodeFSM::executeFSM, this)
        );
    }

    void executeFSM(){
        if(rclcpp::ok() && !fsm_->is_finished()) {
            RCLCPP_INFO(this->get_logger(), "Executing FSM...");
            fsm_->execute();
        } else {
            RCLCPP_ERROR(this->get_logger(), "FSM is not initialized or rclcpp is not ok.");
        }
    }

private:
    std::shared_ptr<Drone> drone_node_;
    std::shared_ptr<VisionNode> vision_node_;
    std::unique_ptr<Fase4FSM> fsm_;
    rclcpp::TimerBase::SharedPtr timer_;
};


template<typename DoubleHandler, typename StringHandler>
std::map<std::string, std::variant<double, std::string>> declareAndGetParameters(
    const std::map<std::string, std::variant<double, std::string>>& defaults,
    DoubleHandler&& double_handler,
    StringHandler&& string_handler) {
        
    std::map<std::string, std::variant<double, std::string>> result;
        
    for (const auto& [name, default_value] : defaults) {
        if (std::holds_alternative<double>(default_value)) {
            result[name] = double_handler(name, std::get<double>(default_value));
        } else if (std::holds_alternative<std::string>(default_value)) {
            result[name] = string_handler(name, std::get<std::string>(default_value));
        }
    }
    
    return result;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto drone = std::make_shared<Drone>();
    auto vision = std::make_shared<VisionNode>();
    auto node = std::make_shared<NodeFSM>(drone, vision);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}