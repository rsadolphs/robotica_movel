// rendering.hpp
#ifndef RENDERING_HPP
#define RENDERING_HPP

#include <cmath>

void* renderingThreadFunction(void* arg);

struct GridInfo {
    float inicio;
    float fim;
    float passo;
};

struct Color {
    float r;
    float g;
    float b;
};

struct RadarPoint {
    float x;
    float y;
    Color color;
    float radius;  // raio do ponto (ex: para markers)
};

#endif // RENDERING_HPP