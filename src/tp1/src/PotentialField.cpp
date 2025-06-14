#include "Mapping.hpp"
#include "rclcpp/rclcpp.hpp"

#include <unistd.h>
#include <vector>

extern std::vector<std::vector<float>> matrizMundo;
extern std::vector<float> offset;
extern float scaleFactor;
extern Position roboPosicao;
extern GridInfo grid;

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

    // Converte posição do robô para índices de célula
    float posX = roboPosicao.x * scaleFactor + offset[0];
    float posY = roboPosicao.y * scaleFactor + offset[1];
    MatrixPosition matPos = findCell(posX, posY, grid.inicio, grid.passo);
    int x0 = matPos.coluna;
    int y0 = matPos.linha;

    // Define limites da janela de 20 células
    int raio = 20;
    int xIni = std::max(1, x0 - raio);
    int xFim = std::min((int)colunas - 2, x0 + raio);
    int yIni = std::max(1, y0 - raio);
    int yFim = std::min((int)linhas - 2, y0 + raio);

    std::vector<std::vector<float>> novoCampo = campoPotencial;
    float erro = std::numeric_limits<float>::max();

    while (erro > epsilon) {
        erro = 0.0f;

        for (int y = yIni; y <= yFim; ++y) {
            for (int x = xIni; x <= xFim; ++x) {
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
        convergeCampo(0.2f);
        
        usleep(200000); // 200ms
    }

    return NULL;
}