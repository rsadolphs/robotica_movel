// graphics.hpp
#ifndef GRAPHICS_HPP
#define GRAPHICS_HPP

#include <cmath>

void* graphicsThreadFunction(void* arg);

struct Position {
    float x, y;
    bool isEqual(const Position& other, float epsilon = 1e-4f) const {
        return std::abs(x - other.x) < epsilon && std::abs(y - other.y) < epsilon;
    }
};
#endif // GRAPHICS_HPP
