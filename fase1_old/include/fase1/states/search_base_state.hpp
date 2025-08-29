#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "fase1/vision_fase1.hpp"
#include "fase1/ArenaPoint.hpp"
#include "fase1/GPS_Handler.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"

class SearchBaseState : public fsm::State {
private:
    std::shared_ptr<Drone> drone;
    std::shared_ptr<VisionNode> vision;  // Fixed typo: vison -> vision

    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr gps_sub_;

    std::vector<ArenaPoint> waypoints;

    Eigen::Vector3d pos;
    Eigen::Vector3d goal;

    Eigen::Vector3d vetor_avanco;
    int counter_path = 0; // essa aqui é a unica que deve ser declarada e ja inicializada aqui
    /*
    isso acontece pq, como o escopo da classe nao morre quando a funcao act termina,
    a variavel counter_path mantem seu valor entre chamadas de act,
    permitindo que o drone continue de onde parou na lista de waypoints.
    O mais importante é que ela serve para multiplicar o vetor E-Origem,
    fazendo o drone ir fazendo o caminho dos waypoints, so que mais para a frente
    */
    int max_path_incrementos;
   

    float max_velocity;
    float position_tolerance;
    float initial_yaw;
    float min_dist_between_bases;

    GPSpos current_gps;
    GPSpos last_gps = EMPTY_GPS;

public:
    SearchBaseState() : fsm::State() {}



    void on_enter(fsm::Blackboard &bb) override {
        auto drone_ptr = bb.get<std::shared_ptr<Drone>>("drone");
        auto vision_ptr = bb.get<std::shared_ptr<VisionNode>>("vision");

        if(drone_ptr == nullptr || vision_ptr == nullptr) return;

        this->drone = *drone_ptr;
        this->vision = *vision_ptr;

        this->drone->log("STATE: SEARCH BASE");

        this->max_velocity = *bb.get<float>("max_vertical_velocity");
        this->position_tolerance = *bb.get<float>("position_tolerance");

        this->pos = this->drone->getLocalPosition();
        this->initial_yaw = this->drone->getOrientation()[2];

        this->waypoints = *bb.get<std::vector<ArenaPoint>>("waypoints");

        this->max_path_incrementos = *bb.get<int>("max_path_incrementos");
        this->vetor_avanco = *bb.get<Eigen::Vector3d>("vetor_avanco");
    
        this->min_dist_between_bases = *bb.get<float>("min_distance_between_bases");

        // gps:
        this->current_gps = EMPTY_GPS;

        rclcpp::QoS gps_qos_sub(10);
        gps_qos_sub.best_effort(); // nao consegui usar o reliable
        gps_qos_sub.durability(rclcpp::DurabilityPolicy::Volatile);

        gps_sub_ = this->drone->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
            "/fmu/out/vehicle_global_position",
            gps_qos_sub,
            [this](const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
                this->gps_callback(msg);
            }
        );
    }

    void gps_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {           
        this->current_gps.lat = static_cast<float>(msg->lat); // double
        this->current_gps.lon = static_cast<float>(msg->lon); // double
        this->current_gps.alt = msg->alt; // float

        this->current_gps.updated = true;
    }

    std::string act(fsm::Blackboard &bb) override {
        (void) bb;

        bb.set<std::shared_ptr<GPSpos>>("current_gps", std::make_shared<GPSpos>(this->current_gps));

        if(this->vision->hasValidBase()){
            if(am_i_far(this->current_gps, this->last_gps, this->min_dist_between_bases)){
                this->drone->log("Base detectada");
                this->drone->setLocalVelocity(0.0, 0.0, 0.0);
                bb.set<Base>("base_detected", this->vision->getCurrentBase());
                
                // CORREÇÃO: Atualizar last_gps para a posição atual
                this->last_gps = this->current_gps;
                bb.set<std::shared_ptr<GPSpos>>("last_gps", std::make_shared<GPSpos>(this->last_gps));
                
                return "BASE DETECTED";
            } else {
                // CORREÇÃO: Se está muito perto, ignorar e continuar navegação
                this->drone->log("Base muito próxima da anterior - ignorando");
                return ""; // Continuar navegando
            }
        }

        this->pos = this->drone->getLocalPosition();

        if(this->counter_path > this->max_path_incrementos){
            this->drone->log("Acabaram os waypoints");
            return "SEARCH ENDED";
        }

        ArenaPoint* p = this->getNextCheckpoint(); // Get pointer to modify is_visited
        if(p == nullptr){
            this->resetCheckpoints();
            return ""; // sem esse return, ocorre segmentation fault, pois p é nullptr
        }

        this->goal = p->coord + (this->vetor_avanco * counter_path); // soma com a multiplicacao do vetor E-Origem pelo counter_path
        
        Eigen::Vector3d diff = this->goal - this->pos;
        this->drone->log("Pos: (" + std::to_string(this->pos.x()) + ", " + std::to_string(this->pos.y()) + ", " + std::to_string(this->pos.z()) + ")");
        this->drone->log("Goal: (" + std::to_string(this->goal.x()) + ", " + std::to_string(this->goal.y()) + ", " + std::to_string(this->goal.z()) + ")");

        if(diff.norm() < this->position_tolerance) {
            this->drone->log("Cheguei no checkpoint: "+p->name);
            p->is_visited = true;
            if(p->name == "E"){
                counter_path += 1;
                this->drone->log("Counter path incremented to: " + std::to_string(counter_path));
            }
        }

        Eigen::Vector3d little_goal =
            pos + (diff.norm() > max_velocity ? diff.normalized() * max_velocity : diff);
        
        this->drone->setLocalPosition(
            little_goal.x(),
            little_goal.y(),
            little_goal.z(),
            this->initial_yaw
        );
        
        return "";
    }

private:
    ArenaPoint* getNextCheckpoint(){
        for(auto& p : this->waypoints){  // Use reference to modify
            if(p.is_visited == false){
                return &p;  // Return pointer
            }
        }
        return nullptr;        
    }

    void resetCheckpoints(){
        this->drone->log("Copiando o caminho padrao novamente");
        for(auto& p : this->waypoints)
            p.is_visited = false;
        this->drone->log("Os waypoints foram resetados com sucesso");
    }
};