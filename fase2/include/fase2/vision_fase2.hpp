#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <custom_msgs/msg/lane_direction.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <memory>
#include <chrono>

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

private:
    // ROS2 subscription
    rclcpp::Subscription<LaneDirectionMsg>::SharedPtr lane_sub_;

    std::chrono::steady_clock::time_point last_detection_time_;
    std::chrono::duration<double> timeout_{5.0};

    void aruco_detection_callback(const ArucoMarkersMsg::SharedPtr msg) {
        // Processar a detecção de marcadores ArUco
    }
};
