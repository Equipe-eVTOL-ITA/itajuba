#pragma once
#include <string>
#include <map>

enum Forma { // fazer com que os indices de deteccao sejam os mesmos no python
    CIRCULO = 0,
    TRIANGULO = 1,
    QUADRADO = 2,
    RETANGULO = 3,
    LOSANGO = 4,
    PENTAGONO = 5,
    HEXAGONO = 6,
    OUTRO = 7,
    NENHUMA_FORMA = 8
};

extern std::map<Forma, std::string> forma_map;

struct Base {
    Forma forma = NENHUMA_FORMA;       // tipo da base (circulo, triangulo, etc.)
    float x;               // coordenada x normalizada [0,1]
    float y;               // coordenada y normalizada [0,1]
    float confidence;      // confiança da detecção
};