// graphics.hpp
#ifndef GRAPHICS_HPP
#define GRAPHICS_HPP

#include <cmath>

void* graphicsThreadFunction(void* arg);

struct Position {
    float x, y, theta;
    bool isEqual(const Position& other, float epsilon = 1e-4f) const {
        return std::abs(x - other.x) < epsilon && std::abs(y - other.y) < epsilon;
    }
};


struct GridInfo {
    float inicio;
    float fim;
    float passo;
};

struct MatrixPosition {
    int linha;
    int coluna;
};

struct CellCenter {
    float x;
    float y;
};

struct Robot {
    MatrixPosition gridPos;
    Position pos;
    CellCenter cellCenter;
    double R;
    float beta = M_PI / 36; //5 graus
};

struct CellRelativeInfo {
    float distancia;
    float anguloRelativo; // em radianos
};

struct CellInfo {
    MatrixPosition celula;
    CellCenter centro;
    CellRelativeInfo relations;
};
#endif // GRAPHICS_HPP
