#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <custom_msgs/msg/aruco_marker_msg.hpp>
#include "fase2/comandos.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <memory>
#include <chrono>

struct ArucoMarker {
    Direcoes dir;
    float x;
    float y;
};

class VisionNode : public rclcpp::Node {
public:
    VisionNode() : Node("fase2_vision") {
        // QoS optimizado para visão computacional
        rclcpp::QoS vision_qos(10);
        vision_qos.best_effort();
        vision_qos.durability(rclcpp::DurabilityPolicy::Volatile);

        // Parâmetro de timeout
        this->declare_parameter<double>("timeout", 5.0);
        timeout_ = std::chrono::duration<double>(this->get_parameter("timeout").as_double());

        // Subscriber para dados de lane detection
        aruco_sub_ = this->create_subscription<ArucoMarkersMsg>(
            "aruco_detection",
            vision_qos,
            [this](const ArucoMarkersMsg::SharedPtr msg) { // callback para detecção de marcadores ArUco
                this->aruco_detection_callback(msg);
            }
        );
    }

    ArucoMarker getCurrentMarker() {
        auto now = std::chrono::steady_clock::now();
        if (now - last_detection_time_ > timeout_) {
            // Se o timeout foi excedido, retornar um marcador inválido
            return ArucoMarker{Direcoes::NENHUMA, 0.0f, 0.0f};
        }
        return current_marker_;
    }

private:
    // ROS2 subscription
    rclcpp::Subscription<ArucoMarkersMsg>::SharedPtr aruco_sub_;

    ArucoMarker current_marker_;

    std::chrono::steady_clock::time_point last_detection_time_;
    std::chrono::duration<double> timeout_{5.0};

    void aruco_detection_callback(const ArucoMarkersMsg::SharedPtr msg) {
        float min_distance_xy = std::numeric_limits<float>::max();
        int closest_id = -1;
        for (size_t i = 0; i < msg->ids.size(); ++i) {
            int id = msg->ids[i];
            const auto& pose = msg->poses[i];
            float distance = std::sqrt(std::pow(pose.position.x, 2) + std::pow(pose.position.y, 2));
            if (distance < min_distance_xy) {
                min_distance_xy = distance;
                closest_id = id;
            }
        }
        if (closest_id != -1) {
            RCLCPP_INFO(this->get_logger(), "Marcador ArUco detectado: ID=%d", closest_id);
            
            this->current_marker_.dir = static_cast<Direcoes>(closest_id);
            this->current_marker_.x = msg->poses[closest_id].position.x * 100.0f; // Convertendo para cm
            this->current_marker_.y = msg->poses[closest_id].position.y * 100.0f;
        }
    }
};
