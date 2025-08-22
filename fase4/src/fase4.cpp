#include <memory>
#include <iostream>
#include <vector>

#include "fsm/fsm.hpp"
#include <rclcpp/rclcpp.hpp>
#include "drone/Drone.hpp"
#include "fase4/vision_fase4.hpp"

// Include all state headers
#include "fase4/arming_state.hpp"
#include "fase4/takeoff_state.hpp"
#include "fase4/follow_lane_state.hpp"
#include "fase4/align_with_circle_state.hpp"
#include "fase4/landing_state.hpp"
#include "fase4/search_lane_state.hpp"
#include "fase4/meia_volta_volver_state.hpp"


template<typename DoubleHandler, typename StringHandler>
std::map<std::string, std::variant<double, std::string>> actOnBlackboardMap(
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

class Fase4FSM : public fsm::FSM {
public:
    Fase4FSM(std::shared_ptr<Drone> drone, std::shared_ptr<VisionNode> vision, const std::map<std::string, std::variant<double, std::string>>& parameters) : fsm::FSM({"ERROR", "FINISHED"}){
        this->blackboard_set<std::shared_ptr<Drone>>("drone", drone);
        this->blackboard_set<std::shared_ptr<VisionNode>>("vision", vision);
        
        // Armazenar parâmetros diretamente no blackboard (já processados do YAML)
        actOnBlackboardMap(
            parameters,
            // Lambda para converter double para float e armazenar no blackboard
            [this](const std::string& name, double value) -> double {
                this->blackboard_set<float>(name, static_cast<float>(value));
                return value;
            },
            // Lambda para armazenar strings no blackboard
            [this](const std::string& name, const std::string& value) -> std::string {
                this->blackboard_set<std::string>(name, value);
                return value;
            }
        );

        // Máquina de Estados
        this->add_state("ARMING", std::make_unique<ArmingState>());
        this->add_state("INITIAL TAKEOFF", std::make_unique<TakeoffState>());
        this->add_state("FOLLOW_LANE", std::make_unique<FollowLaneState>());
        this->add_state("ALIGN_WITH_CIRCLE", std::make_unique<AlignWithCircleState>());
        this->add_state("LAND ON GOAL", std::make_unique<LandingState>());
        this->add_state("GOAL TAKEOFF", std::make_unique<TakeoffState>());
        this->add_state("SEARCH_LANE", std::make_unique<SearchLaneState>());
        this->add_state("MEIA VOLTA VOLVER", std::make_unique<MeiaVoltaVolverState>());

        this->set_initial_state("ARMING");

        // Transições de Estados
        this->add_transitions("ARMING", {
            {"ARMED", "INITIAL TAKEOFF"},
            {"NOT ARMED", "ERROR"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("INITIAL TAKEOFF", {
            {"TAKEOFF COMPLETED", "SEARCH_LANE"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("SEARCH_LANE", {
            {"LANE FOUND", "FOLLOW_LANE"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("FOLLOW_LANE", {
            //{"CIRCLE DETECTED", "ALIGN_WITH_CIRCLE"},
            //{"LANE ENDED", "LAND ON GOAL"},
            {"CIRCLE DETECTED", "LAND ON GOAL"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("ALIGN_WITH_CIRCLE", {
            {"ALIGNED", "LAND ON GOAL"},
            {"TIMEOUT", "LAND ON GOAL"},
            {"CIRCLE LOST", "LAND ON GOAL"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("LAND ON GOAL", {
            {"LANDED", "GOAL TAKEOFF"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("GOAL TAKEOFF", {
            {"TAKEOFF COMPLETED", "MEIA VOLTA VOLVER"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("MEIA VOLTA VOLVER", {
            {"VOLVER COMPLETED", "SEARCH_LANE"},
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
            {"landing_velocity", 0.3},
            {"landing_timeout", 10.0},
            {"max_horizontal_velocity", 0.1},
            {"max_yaw_rate", 2.0},
            {"position_tolerance", 0.1},
            {"max_angle_for_translation", 0.26179}, // 45 graus em radianos
            {"takeoff_height", -1.5},
            {"pid_pos_kp", 1.0},
            {"pid_pos_ki", 0.0},
            {"pid_pos_kd", 0.0},
            {"setpoint", 0.0},
            {"pid_lateral_kp", 0.5},
            {"pid_lateral_ki", 0.0},
            {"pid_lateral_kd", 0.0},
            {"pid_angular_kp", 0.5},
            {"pid_angular_ki", 0.01},
            {"pid_angular_kd", 0.1},
            {"yaw_tolerance", 0.087}, // 5 graus em rad
            {"volver_timeout", 10.0}
        };

        // Create and configure FSM with parameters
        auto parameters = actOnBlackboardMap(
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

        fsm_ = std::make_unique<Fase4FSM>(drone_node_, vision_node_, parameters);
        
        // Agora os parâmetros estão disponíveis no blackboard para os estados
        
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