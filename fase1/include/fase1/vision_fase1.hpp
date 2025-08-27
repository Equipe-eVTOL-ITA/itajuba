#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <custom_msgs/msg/base_detection.hpp>
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <memory>
#include <chrono>

#include "fase1/bases.hpp"

using Detection2DArray = vision_msgs::msg::Detection2DArray;
using Detection2D = vision_msgs::msg::Detection2D;
using BaseDetection = custom_msgs::msg::BaseDetection;

class VisionNode : public rclcpp::Node {
private:
// ROS2 subscription
    rclcpp::Subscription<Detection2DArray>::SharedPtr detection_sub_;
    rclcpp::Publisher<BaseDetection>::SharedPtr base_detection_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr counter_pub_; // publicar o numero de imagens contabilizadas faz com que seja facil do python diferenciar novas bases e salvar as imagens

    Base current_base_;
    std::vector<Base> all_bases_;

    std::chrono::steady_clock::time_point last_detection_time_;
    std::chrono::duration<double> timeout_{5.0};

public:
    VisionNode() : Node("fase1_vision") {
        // QoS optimizado para visão computacional
        rclcpp::QoS vision_qos(10);
        vision_qos.best_effort();
        vision_qos.durability(rclcpp::DurabilityPolicy::Volatile);

        // Parâmetro de timeout
        this->declare_parameter<double>("timeout", 5.0);
        timeout_ = std::chrono::duration<double>(this->get_parameter("timeout").as_double());

        // Subscriber para detecções de bases do base_detector
        detection_sub_ = this->create_subscription<Detection2DArray>(
            "/vertical_camera/classification",
            vision_qos,
            [this](const Detection2DArray::SharedPtr msg) {
                this->detection_callback(msg);
            }
        );

        // Publisher para publicar detecção de base selecionada
        base_detection_pub_ = this->create_publisher<BaseDetection>("/fase1_vision/base_detection", 10);
        
        // Publisher para publicar o número de imagens contabilizadas
        counter_pub_ = this->create_publisher<std_msgs::msg::Int32>("/fase1_vision/counter", 10);

        // Inicializar com base vazia
        current_base_.forma = NENHUMA_FORMA;
        current_base_.x = 0.0f;
        current_base_.y = 0.0f;
        current_base_.confidence = 0.0f;
    }

    Base getCurrentBase() {
        return current_base_;
    }

    bool hasValidBase() {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_detection_time_).count();
        return elapsed < timeout_.count() && current_base_.forma != NENHUMA_FORMA; // verifica se a forma é válida
    }

    std::vector<Base> getAllBases() {
        return all_bases_;
    }

    void publishCounter(int count) {
        std_msgs::msg::Int32 msg;
        msg.data = count;
        counter_pub_->publish(msg);
    }

private:
    void detection_callback(const Detection2DArray::SharedPtr msg) {
        all_bases_.clear(); // limpar o vector de bases para nova detecção

        float min_distance_from_center = std::numeric_limits<float>::max();
        Base closest_base;
        closest_base.forma = NENHUMA_FORMA;

        for (const auto& detection : msg->detections) {
            if (!detection.results.empty()) {
                Base base;
                
                // Convert string class_id to Forma enum
                std::string class_name = detection.results[0].hypothesis.class_id;
                base.forma = NENHUMA_FORMA; // default
                
                // Map string to enum
                for (const auto& [forma, nome] : forma_map) {
                    if (nome == class_name) {
                        base.forma = forma;
                        break;
                    }
                }
                
                base.x = detection.bbox.center.position.x;
                base.y = detection.bbox.center.position.y;
                base.confidence = detection.results[0].hypothesis.score;

                all_bases_.push_back(base);

                // Encontrar a base mais próxima do centro da imagem (0.5, 0.5)
                float distance_from_center = std::sqrt(
                    std::pow(base.x - 0.5f, 2) + 
                    std::pow(base.y - 0.5f, 2)
                );

                if (distance_from_center < min_distance_from_center) {
                    min_distance_from_center = distance_from_center;
                    closest_base = base;
                }
            }
        }

        // Atualizar base atual
        if (closest_base.forma != NENHUMA_FORMA) {
            current_base_ = closest_base;
            last_detection_time_ = std::chrono::steady_clock::now();
            
            // Publicar detecção da base selecionada
            publishBaseDetection(current_base_);
        } else {
            // Nenhuma base detectada
            current_base_.forma = NENHUMA_FORMA;
            current_base_.x = 0.0f;
            current_base_.y = 0.0f;
            current_base_.confidence = 0.0f;
        }
    }

    void publishBaseDetection(const Base& base) {
        BaseDetection msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "camera";
        msg.base_type = forma_map[base.forma];  // Use forma_map to get string
        msg.position.x = base.x;  // coordenadas normalizadas por enquanto
        msg.position.y = base.y;
        msg.position.z = 0.0;     // será calculado pelo FSM se necessário
        msg.confidence = base.confidence;
        msg.detection_id = 0;
        
        base_detection_pub_->publish(msg);
    }
};
