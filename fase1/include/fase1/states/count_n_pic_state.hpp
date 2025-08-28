#pragma once

#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "fase1/vision_fase1.hpp"
#include "fase1/bases.hpp"
#include "fase1/GPS_Handler.hpp"
#include <chrono>
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include <custom_msgs/msg/global_position_msg.hpp>


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

        this->current_gps = EMPTY_GPS;

        rclcpp::QoS gps_qos_pub(10);
        gps_qos_pub.best_effort();
        gps_qos_pub.durability(rclcpp::DurabilityPolicy::Volatile);

        // settando o publisher para as mensagens de gps
        this->global_pos_pub_ = this->drone->create_publisher<custom_msgs::msg::GlobalPositionMsg>(
            "/current_gps_position",
            gps_qos_pub
        );

    }

    std::string act(fsm::Blackboard &bb) override {
        (void)bb;

        auto base = this->vision->getCurrentBase();

        // Corrigir: desreferenciar o shared_ptr corretamente
        auto current_gps_ptr = bb.get<std::shared_ptr<GPSpos>>("current_gps");
        if (!current_gps_ptr) return "";
        this->current_gps = **current_gps_ptr; // Dupla desreferência: *(*current_gps_ptr)

        // Recuperar último GPS do blackboard - corrigir tipos
        auto last_gps_ptr = bb.get<std::shared_ptr<GPSpos>>("last_gps");
        GPSpos last_gps = (last_gps_ptr && *last_gps_ptr) ? **last_gps_ptr : EMPTY_GPS;

        // Só conta se há forma, GPS válido E está longe da última detecção
        if(base.forma != NENHUMA_FORMA && this->current_gps.updated) {
            // Verificar distância antes de contar
            if(am_i_far(this->current_gps, last_gps, 2.0f)) { // usar threshold fixo ou recuperar do blackboard
                this->counter_bases++;

                std::string forma_nome = forma_map[base.forma];
                this->drone->log("Base detectada: " + forma_nome + " (" + std::to_string(this->counter_bases) + "/" + std::to_string(this->max_bases) + ")");

                this->vision->publishCounter(this->counter_bases);

                // Publicando os dados de gps para a rede ros2
                custom_msgs::msg::GlobalPositionMsg gps_msg;
                gps_msg.lat = this->current_gps.lat;
                gps_msg.lon = this->current_gps.lon;
                gps_msg.alt = this->current_gps.alt;
                this->global_pos_pub_->publish(gps_msg);

                return (this->counter_bases >= this->max_bases) ? "ALL BASES FOUND" : "NEXT WAYPOINT";
            } else {
                this->drone->log("Base muito próxima da anterior - não contando");
                return "NEXT WAYPOINT"; // Ignorar esta base e continuar
            }
        }

        return "";
    }
};
