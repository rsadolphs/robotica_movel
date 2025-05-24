// graphics.cpp
#include "graphics.hpp"
#include <GLFW/glfw3.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>

// Variável global ou extern para compartilhar posição do robô
extern Position roboPosicao;
extern Position pontoParedeE;
extern Position pontoParedeD;
extern std::vector<float> sonares;
const std::vector<double> sensorAngles = {-90, -50, -30, -10, 10, 30, 50, 90, 90, 130, 150, 170, -170, -150, -130, -90};


// Histórico de posições
std::vector<Position> caminho;


// Cria Grade e Matrizes
GridInfo grid = {-5.0f, 5.0f, 0.02f};
int size = (grid.fim - grid.inicio) / grid.passo;  
std::vector<std::vector<float>> matrizMundo(size, std::vector<float>(size, 0.0f));
std::vector<std::vector<int>> matrizPath(size, std::vector<int>(size, 0));

float round2(float valor) {
    return std::round(valor * 100.0f) / 100.0f;
}

void desenhaGrade(float inicio, float fim, float passo) {
    glColor3f(0.85f, 0.85f, 0.85f); // cinza claro
    glLineWidth(1.0f);
    glBegin(GL_LINES);

    // Linhas verticais
    for (float x = inicio; x <= fim; x += passo) {
        glVertex2f(x, inicio);
        glVertex2f(x, fim);
    }

    // Linhas horizontais
    for (float y = inicio; y <= fim; y += passo) {
        glVertex2f(inicio, y);
        glVertex2f(fim, y);
    }

    glEnd();
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

void pintaCelulas(const std::vector<std::vector<float>>& matriz, float inicio, float passo, std::tuple<float, float, float> cor) {
    int linhas = matriz.size();
    int colunas = matriz[0].size();
    auto [r, g, b] = cor; 

    for (int i = 0; i < linhas; ++i) {
        for (int j = 0; j < colunas; ++j) {
            float x = inicio + j * passo;
            float y = inicio + i * passo;

            float matVal = matriz[i][j];
            float cellColor = 1.0f * (1 - matVal);

            glColor3f(cellColor, cellColor, cellColor);

            glBegin(GL_QUADS);
                glVertex2f(x, y);
                glVertex2f(x + passo, y);
                glVertex2f(x + passo, y + passo);
                glVertex2f(x, y + passo);
            glEnd();
        
        }
    }
}

MatrixPosition findCell(float x, float y, float inicio, float passo) {
    int coluna = static_cast<int>((x - inicio) / passo);
    int linha  = static_cast<int>((y - inicio) / passo);

    return {linha, coluna};
}

CellCenter centroDaCelula(const MatrixPosition& pos, float inicio, float passo) {
    float x = inicio + pos.coluna * passo + passo / 2.0f;
    float y = inicio + pos.linha  * passo + passo / 2.0f;
    return {x, y};
}

CellRelativeInfo calculaDistanciaEAngulo(
    const CellCenter& centroRobo,
    const CellCenter& centroPonto,
    float yawRobo) 
{
    float dx = centroPonto.x - centroRobo.x;
    float dy = centroPonto.y - centroRobo.y;

    float distancia = std::sqrt(dx * dx + dy * dy);
    float anguloAbsoluto = std::atan2(dy, dx);

    if(anguloAbsoluto < 0){
        anguloAbsoluto = 2 * M_PI + anguloAbsoluto;
    }

    float anguloRelativo = anguloAbsoluto - yawRobo;

    // Converte para graus
    float anguloRelativoGraus = anguloRelativo * 180.0f / M_PI;

    return {distancia, anguloRelativoGraus};
}


bool posicaoValida(const MatrixPosition& pos, int linhas, int colunas) {
    return (pos.linha >= 0 && pos.linha < linhas &&
            pos.coluna >= 0 && pos.coluna < colunas);
}

float bayes(float R, float r, float s, float beta, float alpha, float max, float pOcup) {

    float rangeDetect = 0.2f;

    if(pOcup == 0.0f){ pOcup = 0.5f; } // iteração inicial

    if ((r >= s - rangeDetect) && ( r <= s + rangeDetect)){
        float pSOcup = 0.5f * ( (R-r)/R + (beta-std::abs(alpha)/beta ) )* max;
        float pSVaz = 1.0f - pSOcup;
        float pSOcup_pOcup = pSOcup * pOcup;
        float pSVaz_pVaz = pSVaz * ( 1 - pOcup );

        float pOcupS = (pSOcup_pOcup / (pSOcup_pOcup + pSVaz_pVaz));
        std::cout << "pOcupS" << std::endl;

        return pOcupS;
    }
    else{
        float pVaz = 1 - pOcup;
        float pSVaz = 0.5f * ( (R-r)/R + (beta-std::abs(alpha))/beta ) * max;
        float pSOcup = 1.0f - pSVaz;
        float pSVaz_pVaz = pSVaz * pVaz;
        float pSOcup_pOcup = pSOcup * ( 1 - pVaz );

        float pVazS = (pSVaz_pVaz / (pSVaz_pVaz + pSOcup_pOcup));
        std::cout << "pVazS" << std::endl;
        return 1 - pVazS;
    }

}

void atualizaMatriz(std::vector<std::vector<float>>& matriz, MatrixPosition detection, Robot robot){

    if (detection.linha >= 0 && detection.linha < matriz.size() 
        && detection.coluna >= 0 && detection.coluna < matriz[0].size()) 
    {
        matriz[detection.linha][detection.coluna] += 1;
    }

    int linhas = matriz.size();
    int colunas = matriz[0].size();
    int cont = 0;
    for (int i = 0; i < linhas; ++i) {
        for (int j = 0; j < colunas; ++j) {
            // Locate stuff
            MatrixPosition posAtual = {i, j};
            CellCenter centroAtual = centroDaCelula(posAtual, grid.inicio, grid.passo);
            CellRelativeInfo relations = calculaDistanciaEAngulo(centroAtual, robot.cellCenter, robot.pos.theta);
            CellInfo celulaAtual = {posAtual, centroAtual, relations};
            
            // infos legibilidade
            float r = celulaAtual.relations.distancia;
            float alpha = celulaAtual.relations.anguloRelativo;
            float delta = sensorAngles[0];// * M_PI / 180; // por enquanto apenas um sensor
            float beta = robot.beta;
            float R = 4.0f;
            // Está no cone?
            if ((r <= robot.s) && (alpha >= -beta+delta) && (alpha <= beta+delta)){ // Está no cone!
                
                float pOcup = bayes(R, r, robot.s, beta, alpha,  0.9f,  matriz[i][j]);
                std::cout << "R: " << R << " r: " << r << " s: " << robot.s <<
                        " beta: "  << beta << " alpha: "  << alpha <<
                        " pOcup: "  << pOcup << std::endl;
                matriz[i][j] = pOcup;
                cont += 1;
            }
        }
    }
    std::cout << cont << " celulas atualizadas. " << std::endl;

    salvaMatriz(matriz, "matriz.txt");
    
}

void* graphicsThreadFunction(void* arg) {
    if (!glfwInit()) {
        return NULL;
    }

    int width = 600;
    int height = 600;
    float scaleFactor = 0.1f;

    GLFWwindow* window = glfwCreateWindow(width, height, "Caminho do Robo", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return NULL;
    }

    glfwMakeContextCurrent(window);
    glClearColor(1, 1, 1, 1); // fundo branco

    while (!glfwWindowShouldClose(window)) {
        // Atualiza o caminho
        Position posRobo = {roboPosicao.x * scaleFactor - 0.8, 
                            roboPosicao.y * scaleFactor - 0.7,
                            roboPosicao.theta
        };

        Position posParedeE = {round2(pontoParedeE.x * scaleFactor - 0.8), 
            round2(pontoParedeE.y * scaleFactor - 0.7)};

        Position posParedeD = {round2(pontoParedeD.x * scaleFactor - 0.8), 
            round2(pontoParedeD.y * scaleFactor - 0.7)};

        caminho.push_back(posRobo);  // ATENÇÃO: precisa garantir que acesso à roboPosicao é seguro!


        // Renderizar
        glClear(GL_COLOR_BUFFER_BIT);

        // Adiciona pontos nas matrizes parede e path
        MatrixPosition matPosParedeE = findCell(posParedeE.x, posParedeE.y, grid.inicio, grid.passo);
        MatrixPosition matPosParedeD = findCell(posParedeD.x, posParedeD.y, grid.inicio, grid.passo);
        MatrixPosition matPosRobo = findCell(posRobo.x, posRobo.y, grid.inicio, grid.passo);
        

        int linhas = matrizMundo.size();
        int colunas = matrizMundo[0].size();
        if (posicaoValida(matPosParedeE, linhas, colunas) &&
            posicaoValida(matPosParedeD, linhas, colunas) &&
            posicaoValida(matPosRobo, linhas, colunas) &&
            !sonares.empty()) {

            CellCenter centroCelRobo = centroDaCelula(matPosRobo, grid.inicio, grid.passo);
            Robot robotInfo = {matPosRobo, posRobo, centroCelRobo, sonares[0]};

            atualizaMatriz(matrizMundo, matPosParedeE, robotInfo);
            atualizaMatriz(matrizMundo, matPosParedeD, robotInfo);
        }

        // Desenha a grade
        desenhaGrade(grid.inicio, grid.fim, grid.passo);

        // Pinta células
        pintaCelulas(matrizMundo, grid.inicio, grid.passo, std::make_tuple(0.3f, 0.3f, 0.3f));
        //pintaCelulas(matrizPath, grid.inicio, grid.passo, std::make_tuple(1.0f, 0.0f, 0.0f));

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(-1, 1, -1, 1, -1, 1);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        // Desenha caminho
        glColor3f(1.0f, 0.0f, 0.0f); // vermelho
        glBegin(GL_LINE_STRIP);
        for (auto& pos : caminho) {
            glVertex2f(pos.x, pos.y);
        }
        glEnd();

        float xFinal = 0.0f;
        float yFinal = 0.0f;

        // desenhar sensores
        float sensorLength = 0.0f;
        if (!sonares.empty()) {
            glLineWidth(1.5f);
                for (int i = 0; i <= 15; i++) {
                    glBegin(GL_LINES);
                    glColor3f(0.9f, 0.7f, 0.1f); // laranja escuro
                        if(i==0 || i==7 || i==8|| i==15){
                            glColor3f(0.9f, 0.1f, 0.9f); // magenta
                        }
                        sensorLength = sonares[i] * scaleFactor;
                        xFinal = posRobo.x + cos(posRobo.theta-sensorAngles[i]*M_PI/180) * sensorLength;
                        yFinal = posRobo.y + sin(posRobo.theta-sensorAngles[i]*M_PI/180) * sensorLength;
                        glVertex2f(posRobo.x, posRobo.y);
                        glVertex2f(xFinal, yFinal);
                    glEnd();
                }
        }


        // Desenha o robo
        glColor3f(0.0f, 0.0f, 0.8f); // azul

        float tamanho = 0.02f;  // raio do círculo
        int numSegmentos = 30;  // mais segmentos = círculo mais suave

        glBegin(GL_TRIANGLE_FAN);
            glVertex2f(posRobo.x, posRobo.y); // centro do círculo
            for (int i = 0; i <= numSegmentos; i++) {
                float angulo = 2.0f * M_PI * i / numSegmentos;
                float x = posRobo.x + cos(angulo) * tamanho;
                float y = posRobo.y + sin(angulo) * tamanho;
                glVertex2f(x, y);
            }
        glEnd();

        // Desenhar a linha de direção
        float comprimentoLinha = 0.1f; // comprimento da linha indicadora
        xFinal = posRobo.x + cos(posRobo.theta) * comprimentoLinha;
        yFinal = posRobo.y + sin(posRobo.theta) * comprimentoLinha;

        glColor3f(0.1f, 0.6f, 0.2f); // verde escuro
        glLineWidth(3.0f);
        glBegin(GL_LINES);
            glVertex2f(posRobo.x, posRobo.y);      // início no centro do robô
            glVertex2f(xFinal, yFinal);            // fim na direção de theta
        glEnd();


        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return NULL;
}
