// graphics.cpp
#include "graphics.hpp"
#include <GLFW/glfw3.h>
#include <vector>
#include <cmath>

// Variável global ou extern para compartilhar posição do robô
extern Position roboPosicao;
extern Position pontoParedeE;
extern Position pontoParedeD;
extern std::vector<float> sonares;
const std::vector<double> sensorAngles = {-90, -50, -30, -10, 10, 30, 50, 90, 90, 130, 150, 170, -170, -150, -130, -90};


// Histórico de posições
std::vector<Position> caminho;

struct GridInfo {
    float inicio;
    float fim;
    float passo;
};

// Exemplo de uso:
GridInfo grid = {-5.0f, 5.0f, 0.02f};

int size = (grid.fim - grid.inicio) / grid.passo;  
std::vector<std::vector<int>> matrizMundo(size, std::vector<int>(size, 0));
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

void pintaCelulas(const std::vector<std::vector<int>>& matriz, float inicio, float passo, std::tuple<float, float, float> cor) {
    int linhas = matriz.size();
    int colunas = matriz[0].size();
    auto [r, g, b] = cor; 

    glColor3f(r, g, b);

    for (int i = 0; i < linhas; ++i) {
        for (int j = 0; j < colunas; ++j) {
            if (matriz[i][j] >= 1) {
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
        matriz[linha][coluna] += 1;
    }
}

void* graphicsThreadFunction(void* arg) {
    if (!glfwInit()) {
        return NULL;
    }

    int width = 600;
    int height = 600;
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
        marcaMatriz(posParedeE.x, posParedeE.y, grid.inicio, grid.passo, matrizMundo);
        marcaMatriz(posParedeD.x, posParedeD.y, grid.inicio, grid.passo, matrizMundo);
        marcaMatriz(posRobo.x, posRobo.y, grid.inicio, grid.passo, matrizPath);
        
        // Desenha a grade
        desenhaGrade(grid.inicio, grid.fim, grid.passo);

        // Pinta células
        pintaCelulas(matrizMundo, grid.inicio, grid.passo, std::make_tuple(0.3f, 0.3f, 0.3f));
        pintaCelulas(matrizPath, grid.inicio, grid.passo, std::make_tuple(1.0f, 0.0f, 0.0f));

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
