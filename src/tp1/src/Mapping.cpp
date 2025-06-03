#include "Mapping.hpp"
#include <GLFW/glfw3.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>

// Variável global ou extern para compartilhar posição do robô
extern Position roboPosicao;
extern std::vector<float> sonares;
const std::vector<double> sensorAngles = {-90, -50, -30, -10, 10, 30, 50, 90, 90, 130, 150, 170, -170, -150, -130, -90};

// Histórico de posições
std::vector<Position> caminho;

// Cria Grade e Matrizes
GridInfo grid = {-1.0f, 1.0f, 0.004f};
int size = (grid.fim - grid.inicio) / grid.passo;  
std::vector<std::vector<float>> matrizMundo(size, std::vector<float>(size, 0.5f));
std::vector<std::vector<int>> matrizPath(size, std::vector<int>(size, 0));


float round2(float valor) {
    return std::round(valor * 100.0f) / 100.0f;
}

void salvaMatriz(const std::vector<std::vector<float>>& matriz, const std::string& nomeArquivo) {
    std::ofstream arquivo(nomeArquivo);

    if (!arquivo.is_open()) {
        std::cerr << "Erro ao abrir o arquivo " << nomeArquivo << " para escrita." << std::endl;
        return;
    }

    for (const auto& linha : matriz) {
        for (const auto& valor : linha) {
            arquivo << valor << " ";
        }
        arquivo << "\n";
    }

    arquivo.close();
    std::cout << "Matriz salva com sucesso em " << nomeArquivo << std::endl;
}

MatrixPosition findCell(float x, float y, float inicio, float passo) {
    int coluna = static_cast<int>((x - inicio) / passo);
    int linha  = static_cast<int>((y - inicio) / passo);

    return {linha, coluna};
} // Conversão de ponto para célula da matriz

CellCenter centroDaCelula(const MatrixPosition& pos, float inicio, float passo) {
    float x = inicio + pos.coluna * passo + passo / 2.0f;
    float y = inicio + pos.linha  * passo + passo / 2.0f;
    return {x, y};
} // Ponto central da célula

CellRelativeInfo calculaDistanciaEAngulo(const CellCenter& centroRobo,const CellCenter& centroPonto,float yawRobo) {
    float dx = centroPonto.x - centroRobo.x;
    float dy = centroPonto.y - centroRobo.y;

    float distancia = std::sqrt(dx * dx + dy * dy);
    float anguloAbsoluto = std::atan2(dy, dx);

    // Normaliza para [0, 2PI)
    if(anguloAbsoluto < 0){
        anguloAbsoluto += 2 * M_PI;
    }

    float yawRad = yawRobo;
    if (yawRad < 0) yawRad += 2 * M_PI;

    float anguloRelativo = anguloAbsoluto - yawRad;

    // Normaliza para [-PI, PI]
    if (anguloRelativo > M_PI) anguloRelativo -= 2 * M_PI;
    if (anguloRelativo < -M_PI) anguloRelativo += 2 * M_PI;

    // Converte para graus
    float anguloRelativoGraus = anguloRelativo * 180.0f / M_PI;

    return {distancia, anguloRelativoGraus};
} // Distancia e angulo entre dois pontos (centro de celulas)

bool posicaoValida(const MatrixPosition& pos, int linhas, int colunas) {
    return (pos.linha >= 0 && pos.linha < linhas &&
            pos.coluna >= 0 && pos.coluna < colunas);
} // Verifica se a posição faz parte da matriz 

float bayes(float R, float r, float s, float beta, float alpha, float max, float pOcup) {

    float rangeDetect = 0.01f;

    if ((r >= s - rangeDetect) && ( r <= s + rangeDetect)){
        float pSOcup = 0.5f * ( (R-r)/R + (beta-std::abs(alpha))/beta ) * max;
        float pSVaz = 1.0f - pSOcup;
        float pSOcup_pOcup = pSOcup * pOcup;
        float pSVaz_pVaz = pSVaz * ( 1 - pOcup );

        float pOcupS = (pSOcup_pOcup / (pSOcup_pOcup + pSVaz_pVaz));
        //std::cout << "pOcupS" << std::endl;

        // Debug 
        if (false){
            std::cout 
                << "pSOcup: " << pSOcup << " "
                << "pSVaz: " << pSVaz << " "
                << "pOcup: " << pOcup << " "
                << "pSOcup_pOcup: " << pSOcup_pOcup << " "
                << "pSVaz_pVaz: " << pSVaz_pVaz << " "
                << "pOcupS: " << pOcupS << " "
            << std::endl;
        }

        return pOcupS;
    }
    else{
        float pVaz = 1 - pOcup;
        float pSVaz = 0.5f * ( (R-r)/R + (beta-std::abs(alpha))/beta ) * max;
        float pSOcup = 1.0f - pSVaz;
        float pSVaz_pVaz = pSVaz * pVaz;
        float pSOcup_pOcup = pSOcup * pOcup;

        //float pVazS = (pSVaz_pVaz / (pSVaz_pVaz + pSOcup_pOcup));
        //std::cout << "pVazS" << std::endl;
        //return 1 - pVazS;
        float pOcupS = (pSOcup_pOcup / (pSOcup_pOcup + pSVaz_pVaz));

        // Debug 
        if (false){
            std::cout 
                << "pSOcup: " << pSOcup << " "
                << "pSVaz: " << pSVaz << " "
                << "pOcup: " << pOcup << " "
                << "pSOcup_pOcup: " << pSOcup_pOcup << " "
                << "pSVaz_pVaz: " << pSVaz_pVaz << " "
                << "pOcupS: " << pOcupS << " "
            << std::endl;
        }

        return pOcupS;
    }

} // Calculo da probabilidade de ocupação por bayes

void atualizaMatriz(std::vector<std::vector<float>>& matriz, Robot robot, float sensorAngle) {
    int linhas = matriz.size();
    int colunas = matriz[0].size();

    int cont = 0;
    for (int i = 0; i < linhas; ++i) {
        for (int j = 0; j < colunas; ++j) {
            
            MatrixPosition posAtual = {i, j};
            CellCenter centroAtual = centroDaCelula(posAtual, grid.inicio, grid.passo);

            // Calcula ângulo relativo e distância entre célula e centro do robô
            CellRelativeInfo relations = calculaDistanciaEAngulo(
                robot.cellCenter, 
                centroAtual, 
                robot.pos.theta - sensorAngle // yaw do robô + ângulo do sensor
            );

            float r = relations.distancia;
            float alpha = relations.anguloRelativo;
            float beta = robot.beta;
            float R = 2.0f; // alcance máximo do sensor que considero válido

            // Está no cone do sensor?
            if (robot.s <= R && r <= robot.s && alpha >= -beta && alpha <= beta) {
                float pOcup_atual = matriz[i][j];
                float pOcup = bayes(R, r, robot.s, beta, alpha, 0.95f, pOcup_atual);
                matriz[i][j] = pOcup;
                cont++;

                // Debug 
                if (false){
                    std::cout 
                        << "Celula [" << i << "," << j << "] "
                        << "Robô [" << robot.gridPos.linha << "," << robot.gridPos.coluna << "] "
                        << "Dist: " << r << " "
                        << "s: " << robot.s << " "
                        << "Alpha: " << alpha << " "
                        //<< "Yaw: " << (robot.pos.theta * 180 / M_PI) << " "
                        << "pOcup: " << pOcup << " "
                        << "pOcup_ant: " << pOcup_atual 
                    << std::endl;
                }
            }
        }
    }

    //salvaMatriz(matriz, "matriz.txt");
} // Método de atualização das células da matriz para ocupação