#pragma once

#include <Eigen/Eigen>
#include <map>

namespace Sentido {
    const Eigen::Vector3d FRENTE = Eigen::Vector3d({1.0, 0.0, 0.0});
    const Eigen::Vector3d TRAS = Eigen::Vector3d({-1.0, 0.0, 0.0});
    const Eigen::Vector3d ESQUERDA = Eigen::Vector3d({0.0, -1.0, 0.0});
    const Eigen::Vector3d DIREITA = Eigen::Vector3d({0.0, 1.0, 0.0});
    const Eigen::Vector3d POUSAR = Eigen::Vector3d({0.0, 0.0, 1.0});
    const Eigen::Vector3d NENHUMA = Eigen::Vector3d({0.0, 0.0, 0.0});
}

enum Direcoes {
    FRENTE = 0,
    TRAS,
    ESQUERDA,
    DIREITA,
    POUSAR,
    NENHUMA
};

std::unordered_map<Direcoes, Eigen::Vector3d> dir_to_vel = {
    {FRENTE, Sentido::FRENTE},
    {TRAS, Sentido::TRAS},
    {ESQUERDA, Sentido::ESQUERDA},
    {DIREITA, Sentido::DIREITA},
    {POUSAR, Sentido::POUSAR},
    {NENHUMA, Sentido::NENHUMA}
};

Eigen::Vector3d get_sentido(Direcoes dir) {
    auto it = dir_to_vel.find(dir);
    if (it != dir_to_vel.end()) {
        return it->second;
    }
    return Eigen::Vector3d::Zero(); // Retorna vetor nulo se a direção não for encontrada
}