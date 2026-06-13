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

#endif // RENDERING_HPP