#include <memory>
#include <iostream>
#include <vector>

#include "fsm/fsm.hpp"
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include "drone/Drone.hpp"
#include "fase1/vision_fase1.hpp"
#include "fase1/ArenaPoint.hpp"
#include "fase1/GPS_Handler.hpp"

#include "fase1/states/arming_state.hpp"
#include "fase1/states/takeoff_state.hpp"
#include "fase1/states/landing_state.hpp"
#include "fase1/states/search_base_state.hpp"
#include "fase1/states/align_with_base_state.hpp"
#include "fase1/states/count_n_pic_state.hpp"
#include "fase1/states/return_home_state.hpp"

// Define forma_map here to avoid ODR violations
std::map<Forma, std::string> forma_map = {
    {CIRCULO, "circulo"},
    {TRIANGULO, "triangulo"},
    {QUADRADO, "quadrado"},
    {RETANGULO, "retangulo"},
    {LOSANGO, "losango"},
    {PENTAGONO, "pentagono"},
    {HEXAGONO, "hexagono"},
    {NENHUMA_FORMA, "nenhuma forma"}
};

template<typename IntegerHandler, typename StringHandler, typename BooleanHandler, typename DoubleHandler>
std::map<std::string, std::variant<int, std::string, bool, double>> actOnBlackboardMap(
    const std::map<std::string, std::variant<int, std::string, bool, double>>& defaults,
    IntegerHandler&& integer_handler,
    StringHandler&& string_handler,
    BooleanHandler&& boolean_handler,
    DoubleHandler&& double_handler) {

    std::map<std::string, std::variant<int, std::string, bool, double>> result;

    for (const auto& [name, default_value] : defaults) {
        if(std::holds_alternative<int>(default_value)){
            result[name] = integer_handler(name, std::get<int>(default_value));
        } else if (std::holds_alternative<std::string>(default_value)) {
            result[name] = string_handler(name, std::get<std::string>(default_value));
        } else if (std::holds_alternative<bool>(default_value)) {
            result[name] = boolean_handler(name, std::get<bool>(default_value));
        } else if (std::holds_alternative<double>(default_value)) {
            result[name] = double_handler(name, std::get<double>(default_value));
        }
    }
    
    return result;
}

class Fase1FSM : public fsm::FSM {
public:
    Fase1FSM(std::shared_ptr<Drone> drone, std::shared_ptr<VisionNode> vision, const std::map<std::string, std::variant<int, std::string, bool, double>>& parameters) : fsm::FSM({"ERROR", "FINISHED"}){
        this->blackboard_set<std::shared_ptr<Drone>>("drone", drone);
        this->blackboard_set<std::shared_ptr<VisionNode>>("vision", vision);
        
        actOnBlackboardMap(
            parameters,
            [this](const std::string& name, int value) -> int {
                this->blackboard_set<int>(name, value);
                return value;
            },
            [this](const std::string& name, const std::string& value) -> std::string {
                this->blackboard_set<std::string>(name, value);
                return value;
            },
            [this](const std::string& name, bool value) -> bool {
                this->blackboard_set<bool>(name, value);
                return value;
            },
            [this](const std::string& name, double value) -> double {
                this->blackboard_set<float>(name, static_cast<float>(value));
                return value;
            }
        );

        std::vector<ArenaPoint> waypoints = this->config_waypoints('A', 'E');
        this->blackboard_set<std::vector<ArenaPoint>>("waypoints", waypoints);

        this->blackboard_set<std::shared_ptr<GPSpos>>("last_gps", std::make_shared<GPSpos>(EMPTY_GPS));

        this->add_state("ARMING", std::make_unique<ArmingState>());
        this->add_state("TAKEOFF", std::make_unique<TakeoffState>());
        this->add_state("LANDING", std::make_unique<LandingState>());
        this->add_state("SEARCH BASE", std::make_unique<SearchBaseState>());
        this->add_state("ALIGN WITH BASE", std::make_unique<AlignWithBaseState>());
        this->add_state("COUNT N PIC", std::make_unique<CountNPicState>());
        this->add_state("RETURN HOME", std::make_unique<ReturnHomeState>());

        this->set_initial_state("ARMING");

        // Transições de Estados
        this->add_transitions("ARMING", {
            {"ARMED", "TAKEOFF"},
            {"NOT ARMED", "ERROR"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("TAKEOFF", {
            {"TAKEOFF COMPLETED", "SEARCH BASE"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("LANDING", {
            {"LANDED", "FINISHED"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("SEARCH BASE", {
            {"BASE DETECTED", "ALIGN WITH BASE"},
            {"SEARCH ENDED", "RETURN HOME"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("ALIGN WITH BASE", {
            {"ALIGNED", "COUNT N PIC"},
            {"TIMEOUT", "LANDING"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("COUNT N PIC", {
            {"ALL BASES FOUND", "RETURN HOME"},
            {"NEXT WAYPOINT", "SEARCH BASE"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("RETURN HOME", {
            {"RETURNED", "LANDING"},
            {"SEG FAULT", "ERROR"}
        });
    }

private:
    std::vector<ArenaPoint> config_waypoints(char alpha, char omega){
        std::vector<ArenaPoint> waypoints;

        // Origem
        float x = *this->blackboard_get<float>("fictual_home_x");
        float y = *this->blackboard_get<float>("fictual_home_y");
        float h = *this->blackboard_get<float>("takeoff_height");
        
        waypoints.push_back(ArenaPoint("Origin", x, y, h));

        // Pontos de A até E
        for(char c = alpha; c <= omega; c++){
            std::string char_str(1, c);
            std::string wp_char = "wp_"+char_str+"_";
            
            try {
                x = *this->blackboard_get<float>(wp_char+"x");
                y = *this->blackboard_get<float>(wp_char+"y");
                waypoints.push_back(ArenaPoint(char_str, x, y, h));
            } catch (...) {
                // Se não encontrar o parâmetro, pula para o próximo
                continue;
            }
        }

        int pos_last = waypoints.size() - 1;

        this->blackboard_set<Eigen::Vector3d>("vetor_avanco", waypoints[pos_last].coord - waypoints[0].coord);

        return waypoints;
    }
};

class NodeFSM : public rclcpp::Node {
private:
    std::shared_ptr<Drone> drone_node_;
    std::shared_ptr<VisionNode> vision_node_;
    std::unique_ptr<Fase1FSM> fsm_;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    NodeFSM(std::shared_ptr<Drone> drone, std::shared_ptr<VisionNode> vision) : rclcpp::Node("fase4_fsm"), drone_node_(drone), vision_node_(vision) {
        // Parâmetros alinhados com fsm.yaml (fase4_fsm.ros__parameters)
        std::map<std::string, std::variant<int, std::string, bool, double>> defaults = {
            // Home position parameters
            {"fictual_home_x", 0.0},
            {"fictual_home_y", 0.0},
            {"fictual_home_z", 0.0}, // z aponta para baixo

            {"fig_count", 0}, // contador de figuras detectadas
            {"max_number_of_bases", 6}, // número máximo de bases a serem detectadas
            {"min_distance_between_bases", 3.0}, // distância mínima entre bases para contagem de novas bases

            {"position_tolerance", 0.1}, // tolerância para alinhamento com a base
            {"timeout_duration", 10.0}, // timeout para alinhamento com a base

            // Takeoff parameters
            {"takeoff_height", -1.5},
            {"max_vertical_velocity", 0.5},
            {"initial_takeoff_taken", false},

            // Landing parameters
            {"landing_velocity", 0.3},
            {"landing_timeout", 10.0},

            // PID parameters for positioning
            {"pid_pos_kp", 2.0},
            {"pid_pos_ki", 0.0},
            {"pid_pos_kd", 0.5},
            {"setpoint", 0.0},

            // Pontos waypoints - Padrão básico de varredura
            // A=(-a, 0), B=(-a, b), C=(a, b), D=(a, 2b), E=(0, 2b)
            // Valores alinhados com fsm.yaml (area_half_width = 5.0)
            {"max_path_incrementos", 2},
            {"wp_A_x", -5.0}, {"wp_A_y", 0.0},
            {"wp_B_x", -5.0}, {"wp_B_y", 1.0},
            {"wp_C_x", 5.0}, {"wp_C_y", 1.0},
            {"wp_D_x", 5.0}, {"wp_D_y", 2.0},
            {"wp_E_x", 0.0}, {"wp_E_y", 2.0}
        };

        auto parameters = actOnBlackboardMap(
            defaults,

            [this](const std::string& name, int default_value) -> int {
                this->declare_parameter(name, default_value);
                return this->get_parameter(name).as_int();
            },
            [this](const std::string& name, const std::string& default_value) -> std::string {
                this->declare_parameter(name, default_value);
                return this->get_parameter(name).as_string();
            },
            [this](const std::string& name, bool default_value) -> bool {
                this->declare_parameter(name, default_value);
                return this->get_parameter(name).as_bool();
            },
            [this](const std::string& name, double default_value) -> double {
                this->declare_parameter(name, default_value);
                return this->get_parameter(name).as_double();
            }
        );

        fsm_ = std::make_unique<Fase1FSM>(drone_node_, vision_node_, parameters);
        
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
