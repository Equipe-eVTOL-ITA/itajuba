#ifndef FASE2_COMANDOS_HPP
#define FASE2_COMANDOS_HPP

#include <Eigen/Eigen>

namespace Velocidade {
    const Eigen::Vector3d FRENTE = Eigen::Vector3d({1.0, 0.0, 0.0});
    const Eigen::Vector3d TRAS = Eigen::Vector3d({-1.0, 0.0, 0.0});
    const Eigen::Vector3d ESQUERDA = Eigen::Vector3d({0.0, -1.0, 0.0});
    const Eigen::Vector3d DIREITA = Eigen::Vector3d({0.0, 1.0, 0.0});
    const Eigen::Vector3d POUSAR = Eigen::Vector3d({0.0, 0.0, 0.0});
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

extern const std::map<Direcoes, Eigen::Vector3d> _DIRECTIONS;
inline const Eigen::Vector3d& get_direction_vector(Direcoes dir) {
    auto it = _DIRECTIONS.find(dir);
    if (it == _DIRECTIONS.end()) {
        static const Eigen::Vector3d zero_vec(0.0, 0.0, 0.0);
        return zero_vec;
    }
    return it->second;
}

// o metodo DIRECTIONS.at(direcao) retorna uma referencia para um Eigen::Vector3d

#endif