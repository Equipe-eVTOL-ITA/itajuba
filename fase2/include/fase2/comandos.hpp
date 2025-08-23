#ifndef FASE2_COMANDOS_HPP
#define FASE2_COMANDOS_HPP

#include <Eigen/Eigen>

namespace Velocidade {
    constexpr Eigen::Vector3d FRENTE = Eigen::Vector3d({1.0, 0.0, 0.0});
    constexpr Eigen::Vector3d TRAS = Eigen::Vector3d({-1.0, 0.0, 0.0});
    constexpr Eigen::Vector3d ESQUERDA = Eigen::Vector3d({0.0, -1.0, 0.0});
    constexpr Eigen::Vector3d DIREITA = Eigen::Vector3d({0.0, 1.0, 0.0});
    constexpr Eigen::Vector3d NENHUMA = Eigen::Vector3d({0.0, 0.0, 0.0});
}

enum Direcoes {
    FRENTE = 0,
    TRAS,
    ESQUERDA,
    DIREITA,
    NENHUMA
};

constexpr std::map<Direcoes, Eigen::Vector3d> DIRECTIONS = {
    {FRENTE, Velocidade::FRENTE},
    {TRAS, Velocidade::TRAS},
    {ESQUERDA, Velocidade::ESQUERDA},
    {DIREITA, Velocidade::DIREITA},
    {NENHUMA, Velocidade::NENHUMA}
};

#endif