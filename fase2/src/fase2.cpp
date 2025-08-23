#include <memory>
#include <iostream>
#include <vector>

#include "fsm/fsm.hpp"
#include <rclcpp/rclcpp.hpp>
#include "drone/Drone.hpp"
#include "fase2/vision_fase4.hpp"

#include "fase2/arming_state.hpp"
#include "fase2/takeoff_state.hpp"
#include "fase2/aruco_search_state.hpp"
#include "fase2/aruco_align_state.hpp"
#include "fase2/aruco_traverse_state.hpp"
#include "fase2/landing_state.hpp"

template<typename DoubleHandler, typename StringHandler, typename BooleanHandler>
std::map<std::string, std::variant<double, std::string, bool>> actOnBlackboardMap(
    const std::map<std::string, std::variant<double, std::string, bool>>& defaults,
    DoubleHandler&& double_handler,
    StringHandler&& string_handler,
    BooleanHandler&& boolean_handler) {

    std::map<std::string, std::variant<double, std::string, bool>> result;

    for (const auto& [name, default_value] : defaults) {
        if (std::holds_alternative<double>(default_value)) {
            result[name] = double_handler(name, std::get<double>(default_value));
        } else if (std::holds_alternative<std::string>(default_value)) {
            result[name] = string_handler(name, std::get<std::string>(default_value));
        } else if (std::holds_alternative<bool>(default_value)) {
            result[name] = boolean_handler(name, std::get<bool>(default_value));
        }
    }
    
    return result;
}

class Fase4FSM : public fsm::FSM {
public:
    Fase4FSM(std::shared_ptr<Drone> drone, std::shared_ptr<VisionNode> vision, const std::map<std::string, std::variant<double, std::string, bool>>& parameters) : fsm::FSM({"ERROR", "FINISHED"}){
        this->blackboard_set<std::shared_ptr<Drone>>("drone", drone);
        this->blackboard_set<std::shared_ptr<VisionNode>>("vision", vision);
        
        actOnBlackboardMap(
            parameters,
            [this](const std::string& name, double value) -> double {
                this->blackboard_set<float>(name, static_cast<float>(value));
                return value;
            },
            [this](const std::string& name, const std::string& value) -> std::string {
                this->blackboard_set<std::string>(name, value);
                return value;
            },
            [this](const std::string& name, bool value) -> bool {
                this->blackboard_set<bool>(name, value);
                return value;
            }
        );

        this->add_state("ARMING", std::make_unique<ArmingState>());
        this->add_state("TAKEOFF", std::make_unique<TakeoffState>());
        this->add_state("ARUCO TRAVERSE", std::make_unique<ArucoTraverseState>());
        this->add_state("ARUCO ALIGN", std::make_unique<ArucoAlignState>());
        this->add_state("ARUCO SEARCH", std::make_unique<ArucoSearchState>());
        this->add_state("LANDING", std::make_unique<LandingState>());

        this->set_initial_state("ARMING");

        // Transições de Estados
        this->add_transitions("ARMING", {
            {"ARMED", "TAKEOFF"},
            {"NOT ARMED", "ERROR"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("TAKEOFF", {
            {"TAKEOFF COMPLETED", "ARUCO TRAVERSE"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("ARUCO ALIGN", {
            {"ALIGNED", "ARUCO TRAVERSE"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("ARUCO TRAVERSE", {
            {"NO MARKER", "SEARCH ARUCO"},
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
        // Parâmetros alinhados com fsm.yaml (fase4_fsm.ros__parameters)
        std::map<std::string, std::variant<double, std::string, bool>> defaults = {
            {"fictual_home_x", 0.0},
            {"fictual_home_y", 0.0},
            {"fictual_home_z", 0.0}, // z aponta para baixo

            {"takeoff_height", -1.5},
            {"max_vertical_velocity", 0.5},
            {"landing_velocity", 0.3},
            {"landing_timeout", 10.0},
            {"max_horizontal_velocity", 0.5},
            {"max_yaw_rate", 1.0},
            {"position_tolerance", 0.1},

            {"pid_pos_kp", 2.0},
            {"pid_pos_ki", 0.0},
            {"pid_pos_kd", 0.5},
            {"setpoint", 0.0}
        };

        auto parameters = actOnBlackboardMap(
            defaults,
            [this](const std::string& name, double default_value) -> double {
                this->declare_parameter(name, default_value);
                return this->get_parameter(name).as_double();
            },
            [this](const std::string& name, const std::string& default_value) -> std::string {
                this->declare_parameter(name, default_value);
                return this->get_parameter(name).as_string();
            },
            [this](const std::string& name, bool default_value) -> bool {
                this->declare_parameter(name, default_value);
                return this->get_parameter(name).as_bool();
            }
        );

        fsm_ = std::make_unique<Fase4FSM>(drone_node_, vision_node_, parameters);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&NodeFSM::executeFSM, this)
        );
    }

    void executeFSM(){
        if(rclcpp::ok() && !fsm_->is_finished())
            fsm_->execute();
        else
            rclcpp::shutdown();
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);    
    rclcpp::executors::MultiThreadedExecutor executor;

    auto drone = std::make_shared<Drone>();
    auto vision = std::make_shared<VisionNode>();
    auto fsm_node = std::make_shared<NodeFSM>(drone, vision);

    executor.add_node(fsm_node);
    executor.add_node(drone);
    executor.add_node(vision);

    executor.spin();
    
    rclcpp::shutdown();
    
    return 0;
}
