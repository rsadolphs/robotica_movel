#include "Mapping.hpp"
#include "rclcpp/rclcpp.hpp"

#include <unistd.h>
#include <vector>

extern std::vector<std::vector<float>> matrizMundo;

std::vector<std::vector<float>> campoPotencial;
std::vector<std::vector<bool>> knownRegion;

void initMatrixes() {
    size_t linhas = matrizMundo.size();
    size_t colunas = linhas > 0 ? matrizMundo[0].size() : 0;

    knownRegion.resize(linhas);
    campoPotencial.resize(linhas);

    for (size_t i = 0; i < linhas; ++i) {
        knownRegion[i].resize(colunas, false);
        campoPotencial[i].resize(colunas, 0.0f);
    }
} 

void atualizaCampoPotencial() {
    for (size_t y = 0; y < knownRegion.size(); ++y) {
        for (size_t x = 0; x < knownRegion[y].size(); ++x) {
            if (knownRegion[y][x] && matrizMundo[y][x] > 10.0f) {
                campoPotencial[y][x] = 1.0f;
            }
        }
    }
}

void convergeCampo(float epsilon) {
    if (campoPotencial.empty()) return;

    size_t linhas = campoPotencial.size();
    size_t colunas = campoPotencial[0].size();
    std::vector<std::vector<float>> novoCampo = campoPotencial;
    float erro = std::numeric_limits<float>::max();

    while (erro > epsilon) {
        erro = 0.0f;

        for (size_t y = 1; y < linhas - 1; ++y) {
            for (size_t x = 1; x < colunas - 1; ++x) {
                if (campoPotencial[y][x] != 1.0f) {
                    float valorNovo = 0.25f * (
                        campoPotencial[y - 1][x] +
                        campoPotencial[y + 1][x] +
                        campoPotencial[y][x - 1] +
                        campoPotencial[y][x + 1]
                    );

                    erro += std::pow(campoPotencial[y][x] - valorNovo, 2);
                    novoCampo[y][x] = valorNovo;
                }
                
            }
        }

        campoPotencial = novoCampo;
    }
}

void* potentialFieldThreadFunction(void* arg) {

    initMatrixes();

    while (rclcpp::ok()) {
        atualizaCampoPotencial();
        convergeCampo(0.01f);
        
        usleep(100000); // 100ms
    }

    return NULL;
}