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
float yawGradienteConv = 0.0f;

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
    
    std::vector<std::vector<float>> novoCampo = campoPotencial;
    float erro = std::numeric_limits<float>::max();

    int cont = 0;
    //while (erro > epsilon) {
    while (cont < 80) {
        erro = 0.0f;
        cont++;
        for (int y = 0; y <= linhas - 1; ++y) {
            for (int x = 0; x <= colunas - 1; ++x) {
                if (campoPotencial[y][x] != 1.0f && knownRegion[y][x]) {
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

void resetCampoPotencial() {
    for (auto& linha : campoPotencial) {
        std::fill(linha.begin(), linha.end(), 0.0f);
    }
}

float calculaYawGradiente(
    const std::vector<std::vector<float>>& campoPotencial,
    float posX, float posY
) {

    // Verifica se estamos em uma posição válida (não na borda)
    int largura = campoPotencial[0].size();
    int altura  = campoPotencial.size();
    
    MatrixPosition matPos = findCell(posX, posY, grid.inicio, grid.passo);

    int x = matPos.coluna;
    int y = matPos.linha;

    if (x <= 0 || x >= largura - 1 || y <= 0 || y >= altura - 1) {
        // Fora da área onde dá pra calcular o gradiente central
        std::cout << "FORA DA AREA DE MAPEAMENTO" << std::endl;
        std::cout << "SIZE: " << largura << "," << altura << std::endl;
        std::cout << "POS: " << x << "," << y << std::endl;
        return 0.0f;
    }


    // Gradiente com diferenças centrais
    float dx = campoPotencial[y][x + 1] - campoPotencial[y][x - 1];
    float dy = campoPotencial[y + 1][x] - campoPotencial[y - 1][x];

    // Direção do declive (gradiente descendente)
    float yaw = std::atan2(-dy, -dx); // negativo pois queremos a direção da descida

    return yaw; // em radianos
}

void* potentialFieldThreadFunction(void* arg) {

    initMatrixes();

    while (rclcpp::ok()) {
        resetCampoPotencial();
        atualizaCampoPotencial();
        convergeCampo(0.01f);
        float x = roboPosicao.x * scaleFactor;
        float y = roboPosicao.y * scaleFactor;
        yawGradienteConv = calculaYawGradiente(campoPotencial, x, y);

        usleep(100000); // 200ms
    }

    return NULL;
}