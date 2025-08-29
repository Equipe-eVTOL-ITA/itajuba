#pragma once

#include <Eigen/Eigen>
#include <string>

class ArenaPoint {
public:
    std::string name = "default";
    Eigen::Vector3d coord; // Posição no espaço (x, y, z)
    bool is_visited; // Indica se o ponto já foi visitado

    ArenaPoint(std::string name, float x, float y, float z) {
        this->name = name;
        this->coord = Eigen::Vector3d(x, y, z);
        this->is_visited = false; // Inicializa como não visitado por padrão
    }
};
