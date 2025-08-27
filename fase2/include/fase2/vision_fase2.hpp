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

using ArucoMarkerMsg = custom_msgs::msg::ArucoMarkerMsg;

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

        // Subscriber para dados de lane detection
        aruco_sub_ = this->create_subscription<ArucoMarkerMsg>(
            "aruco_detection",
            vision_qos,
            [this](const ArucoMarkerMsg::SharedPtr msg) { // callback para detecção de marcadores ArUco
                this->aruco_detection_callback(msg);
            }
        );
    }

    ArucoMarker getCurrentMarker() {
        return current_marker_;
    }

private:
    // ROS2 subscription
    rclcpp::Subscription<ArucoMarkerMsg>::SharedPtr aruco_sub_;

    ArucoMarker current_marker_;

    std::chrono::steady_clock::time_point last_detection_time_;
    std::chrono::duration<double> timeout_{5.0};

    void aruco_detection_callback(const ArucoMarkerMsg::SharedPtr msg) {
        float min_distance_xy = std::numeric_limits<float>::max();
        int closest_id_index = -1;
        int closest_id = -1;
        for (size_t i = 0; i < msg->ids.size(); ++i) {
            int id = msg->ids[i];
            const auto& pose = msg->poses[i];
            float distance = std::sqrt(std::pow(pose.position.x-0.5f, 2) + std::pow(pose.position.y-0.5f, 2));
            if (distance < min_distance_xy) {
                min_distance_xy = distance;
                closest_id = id;
                closest_id_index = i;
            }
        }

        if (closest_id != -1) {
            this->current_marker_.dir = static_cast<Direcoes>(closest_id);
            this->current_marker_.x = msg->poses[closest_id_index].position.x * 100.0f; // Convertendo para cm
            this->current_marker_.y = msg->poses[closest_id_index].position.y * 100.0f;
        } else {
            this->current_marker_.dir = Direcoes::NENHUMA;
            this->current_marker_.x = 0.0f;
            this->current_marker_.y = 0.0f;
        }
    }
};
