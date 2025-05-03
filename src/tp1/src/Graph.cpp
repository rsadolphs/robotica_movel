// graphics.cpp
#include "graphics.hpp"
#include <GLFW/glfw3.h>
#include <vector>
#include <cmath>

// Variável global ou extern para compartilhar posição do robô
extern Position roboPosicao;
extern Position pontoParede;
extern Position pontoParedeE;
extern Position pontoParedeD;

// Histórico de posições
std::vector<Position> caminho;
std::vector<Position> paredes;
std::vector<Position> paredesE;
std::vector<Position> paredesD;

struct GridInfo {
    float inicio;
    float fim;
    float passo;
};

// Exemplo de uso:
GridInfo grid = {-5.0f, 5.0f, 0.02f};

int size = (grid.fim - grid.inicio) / grid.passo;  
std::vector<std::vector<int>> matrizMundo(size, std::vector<int>(size, 0));

float round2(float valor) {
    return std::round(valor * 100.0f) / 100.0f;
}

void desenhaGrade(float inicio, float fim, float passo) {
    glColor3f(0.85f, 0.85f, 0.85f); // cinza claro
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

void pintaCelulas(const std::vector<std::vector<int>>& matriz, float inicio, float passo) {
    int linhas = matriz.size();
    int colunas = matriz[0].size();

    glColor3f(0.3f, 0.3f, 0.3f);  // cinza escuro para células ocupadas

    for (int i = 0; i < linhas; ++i) {
        for (int j = 0; j < colunas; ++j) {
            if (matriz[i][j] == 1) {
                float x = inicio + j * passo;
                float y = inicio + i * passo;

                glBegin(GL_QUADS);
                    glVertex2f(x, y);
                    glVertex2f(x + passo, y);
                    glVertex2f(x + passo, y + passo);
                    glVertex2f(x, y + passo);
                glEnd();
            }
        }
    }
}

void marcaMatriz(float x, float y, float inicio, float passo, std::vector<std::vector<int>>& matriz) {
    int coluna = static_cast<int>((x - inicio) / passo);
    int linha  = static_cast<int>((y - inicio) / passo);

    if (linha >= 0 && linha < matriz.size() && coluna >= 0 && coluna < matriz[0].size()) {
        matriz[linha][coluna] = 1;
    }
}

void* graphicsThreadFunction(void* arg) {
    if (!glfwInit()) {
        return NULL;
    }

    int width = 400;
    int height = 300;
    float scaleFactor = 0.09f;

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
                            roboPosicao.y * scaleFactor - 0.7};
        Position posParede = {round2(pontoParede.x * scaleFactor - 0.8), 
                              round2(pontoParede.y * scaleFactor - 0.7)};

        Position posParedeE = {round2(pontoParedeE.x * scaleFactor - 0.8), 
            round2(pontoParedeE.y * scaleFactor - 0.7)};

        Position posParedeD = {round2(pontoParedeD.x * scaleFactor - 0.8), 
            round2(pontoParedeD.y * scaleFactor - 0.7)};

        caminho.push_back(posRobo);  // ATENÇÃO: precisa garantir que acesso à roboPosicao é seguro!
        paredes.push_back(posParede);
        paredesE.push_back(posParedeE);
        paredesD.push_back(posParedeD);

        // Renderizar
        glClear(GL_COLOR_BUFFER_BIT);

        marcaMatriz(posParedeE.x, posParedeE.y, grid.inicio, grid.passo, matrizMundo);
        marcaMatriz(posParedeD.x, posParedeD.y, grid.inicio, grid.passo, matrizMundo);
        desenhaGrade(grid.inicio, grid.fim, grid.passo);
        pintaCelulas(matrizMundo, grid.inicio, grid.passo);

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

        // Desenha o robo
        // Cor do robô
        glColor3f(0.0f, 0.0f, 0.8f); // azul

        float tamanho = 0.02f;  // metade do lado do quadrado

        // Desenhar o quadrado (corpo do robô)
        glBegin(GL_QUADS);
            glVertex2f(posRobo.x - tamanho, posRobo.y - tamanho);
            glVertex2f(posRobo.x + tamanho, posRobo.y - tamanho);
            glVertex2f(posRobo.x + tamanho, posRobo.y + tamanho);
            glVertex2f(posRobo.x - tamanho, posRobo.y + tamanho);
        glEnd();

        // Desenha as paredes
        glColor3f(0.0f, 0.0f, 0.0f);  // preto
        glBegin(GL_POINTS);
        for (auto& parede : paredes) {
            glVertex2f(parede.x, parede.y);  // desenha cada ponto das paredes
        }
        glEnd();

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return NULL;
}
