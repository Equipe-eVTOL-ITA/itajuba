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

typedef custom_msgs::msg::LaneDirection LaneDirectionMsg;

struct LaneData {
    float theta;           // Ângulo da faixa (em radianos)
    int32_t x_centroid;    // Centro X da faixa (pixels)
    int32_t y_centroid;    // Centro Y da faixa (pixels)
    int64_t timestamp;     // Timestamp da detecção
};

class VisionNode : public rclcpp::Node {
public:
    VisionNode() : Node("fase4_vision") {
        // QoS optimizado para visão computacional
        rclcpp::QoS vision_qos(10);
        vision_qos.best_effort();
        vision_qos.durability(rclcpp::DurabilityPolicy::Volatile);

        // Parâmetro de timeout
        this->declare_parameter<double>("timeout", 5.0);
        timeout_ = std::chrono::duration<double>(this->get_parameter("timeout").as_double());

        // Subscriber para dados de lane detection
        lane_sub_ = this->create_subscription<LaneDirectionMsg>(
            "lane_detection",
            vision_qos,
            [this](const LaneDirectionMsg::SharedPtr msg) { // callback para detecção de faixa
                this->lane_detection_callback(msg);
            }
        );

        RCLCPP_INFO(this->get_logger(), "Lane Vision Node initialized successfully");
    }

    // Métodos públicos para acessar os dados da faixa
    bool isLaneDetected() {
        double time_since_last = this->getLastDetectionTime();
        return has_lane_detection_ && (time_since_last <= timeout_.count());
    }

    LaneData getCurrentLaneData() {
        return current_lane_data_;
    }

    double getLastDetectionTime() {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(now - last_detection_time_).count();
    }

    // Retorna o ângulo da faixa em radianos
    float getLaneAngle() {
        return current_lane_data_.theta;
    }

    // Retorna o centro da faixa em pixels
    Eigen::Vector2i getLaneCentroid() {
        return Eigen::Vector2i(current_lane_data_.x_centroid, current_lane_data_.y_centroid);
    }

private:
    // ROS2 subscription
    rclcpp::Subscription<LaneDirectionMsg>::SharedPtr lane_sub_;

    // Estado da detecção
    LaneData current_lane_data_;
    bool has_lane_detection_{false};
    std::chrono::steady_clock::time_point last_detection_time_;
    std::chrono::duration<double> timeout_{5.0};

    void lane_detection_callback(const LaneDirectionMsg::SharedPtr msg) {
        // Atualiza timestamp
        last_detection_time_ = std::chrono::steady_clock::now();

        // Verifica se há detecção válida
        if (msg->x_centroid == 0 && msg->y_centroid == 0 && msg->theta == 0.0f) {
            has_lane_detection_ = false;
            RCLCPP_DEBUG(this->get_logger(), "No lane detected");
            return;
        }

        // Armazena dados da faixa
        current_lane_data_.theta = msg->theta;
        current_lane_data_.x_centroid = msg->x_centroid;
        current_lane_data_.y_centroid = msg->y_centroid;
        current_lane_data_.timestamp = this->get_clock()->now().nanoseconds();
        
        has_lane_detection_ = true;
        
        RCLCPP_DEBUG(this->get_logger(), 
                    "Lane detected - Theta: %.3f, Center: (%d, %d)", 
                    msg->theta, msg->x_centroid, msg->y_centroid);
    }
};
