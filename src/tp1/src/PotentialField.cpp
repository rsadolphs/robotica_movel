#include "Mapping.hpp"
#include "rclcpp/rclcpp.hpp"

#include <unistd.h>
#include <vector>

extern std::vector<std::vector<float>> matrizMundo;

std::vector<std::vector<float>> campoPotencial = matrizMundo;
std::vector<std::vector<bool>> knownRegion;

void initKnownRegion() {
    knownRegion.resize(matrizMundo.size());
    for (size_t i = 0; i < matrizMundo.size(); ++i) {
        knownRegion[i].resize(matrizMundo[i].size(), false);
    }
}

void* potentialFieldThreadFunction(void* arg) {

    initKnownRegion();

    while (rclcpp::ok()) {

        // Pequena pausa para não sobrecarregar a CPU
        usleep(1000000); // 1000ms
    }

    return NULL;
}