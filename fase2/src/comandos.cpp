// Definição global do map DIRECTIONS para evitar ODR violation
#include "fase2/comandos.hpp"

const std::map<Direcoes, Eigen::Vector3d> _DIRECTIONS = {
    {FRENTE, Velocidade::FRENTE},
    {TRAS, Velocidade::TRAS},
    {ESQUERDA, Velocidade::ESQUERDA},
    {DIREITA, Velocidade::DIREITA},
    {POUSAR, Velocidade::POUSAR},
    {NENHUMA, Velocidade::NENHUMA}
};
