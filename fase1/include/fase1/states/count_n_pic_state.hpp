#pragma once

#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "fase1/vision_fase1.hpp"
#include "fase1/bases.hpp"
#include <chrono>
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include <custom_msgs/msg/global_position_msg.hpp>

typedef struct{
    float lat;
    float lon;
    float alt;
    bool updated;
} GPSpos;

class CountNPicState : public fsm::State {
private:
    std::shared_ptr<Drone> drone;
    std::shared_ptr<VisionNode> vision;

    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr gps_sub_;
    rclcpp::Publisher<custom_msgs::msg::GlobalPositionMsg>::SharedPtr global_pos_pub_;

    Base base;

    int counter_bases;
    int max_bases;

    GPSpos current_gps;

public:
    CountNPicState() : fsm::State() {}

    void on_enter(fsm::Blackboard &bb) override {
        auto drone_ptr = bb.get<std::shared_ptr<Drone>>("drone");
        auto vision_ptr = bb.get<std::shared_ptr<VisionNode>>("vision");

        if(drone_ptr == nullptr || vision_ptr == nullptr) return;

        this->drone = *drone_ptr;
        this->vision = *vision_ptr;

        this->drone->log("STATE: COUNT N PIC");

        this->counter_bases = 0;

        this->max_bases = *bb.get<int>("max_number_of_bases");

        // QoS para garantir que recebo os dados do GPS
        rclcpp::QoS gps_qos(10);
        gps_qos.reliable();
        gps_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

        // Como CountNPicState não é um nó ROS2, vamos usar o drone para criar subscribers/publishers
        gps_sub_ = this->drone->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
            "/fmu/out/vehicle_global_position",
            gps_qos,
            [this](const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
                this->gps_callback(msg);
            }
        );

        this->current_gps = {0.0, 0.0, 0.0, false};

        // settando o publisher para as mensagens de gps
        this->global_pos_pub_ = this->drone->create_publisher<custom_msgs::msg::GlobalPositionMsg>(
            "/current_gps_position",
            gps_qos // utilizando o mesmo QoS que o subscriber
        );

    }

    void gps_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
        // Processar dados de GPS aqui
        //if (msg->lat_lon_valid && msg->alt_valid) {
            // Latitude e longitude em graus, altitude em metros            
            this->current_gps.lat = static_cast<float>(msg->lat); // double
            this->current_gps.lon = static_cast<float>(msg->lon); // double
            this->current_gps.alt = msg->alt; // float

            this->current_gps.updated = true;
        //}
    }

    std::string act(fsm::Blackboard &bb) override {
        (void)bb;

        auto base = this->vision->getCurrentBase();

        // so roda se ha forma e se pegou o sinal de gps
        if(base.forma != NENHUMA_FORMA && this->current_gps.updated) {
            this->counter_bases++;

            std::string forma_nome = forma_map[base.forma];
            this->drone->log("Base detectada: " + forma_nome + " (Total: " + std::to_string(this->counter_bases) + ")");

            this->vision->publishCounter(this->counter_bases);

            // Publicando os dados de gps para a rede ros2
            custom_msgs::msg::GlobalPositionMsg gps_msg;
            gps_msg.lat = this->current_gps.lat;
            gps_msg.lon = this->current_gps.lon;
            gps_msg.alt = this->current_gps.alt;
            this->global_pos_pub_->publish(gps_msg);

            return (this->counter_bases >= this->max_bases) ? "ALL BASES FOUND" : "NEXT WAYPOINT";
        }

        return "";
    }
};
