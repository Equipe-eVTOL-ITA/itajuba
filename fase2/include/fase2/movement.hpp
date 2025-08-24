#pragma once

#include <Eigen/Eigen>
#include "comandos.hpp"
#include "drone/Drone.hpp"
#include "transformations.hpp"

void move_local(std::shared_ptr<Drone> drone, Eigen::Vector3d direction, float speed) {
    if (drone == nullptr) return;

    Eigen::Vector3d local_velocity = direction * speed;
    Eigen::Vector3d adjusted_velocity = adjust_velocity_using_yaw(local_velocity, drone->getOrientation()[2]);
    drone->setLocalVelocity(adjusted_velocity.x(), adjusted_velocity.y(), adjusted_velocity.z(), 0.0f);
}