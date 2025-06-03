// graphics.cpp
#include "graphics.hpp"
#include "Mapping.hpp"
#include <GLFW/glfw3.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <algorithm>

// Variável global ou extern para compartilhar posição do robô
extern Position roboPosicao;
extern std::vector<float> sonares;
const std::vector<double> sensorAngles = {-90, -50, -30, -10, 10, 30, 50, 90, 90, 130, 150, 170, -170, -150, -130, -90};

// Histórico de posições
extern std::vector<Position> caminho;

// Cria Grade e Matrizes
extern GridInfo grid;
extern int size;  
extern std::vector<std::vector<float>> matrizMundo;
extern std::vector<std::vector<int>> matrizPath;


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

void pintaCelulas(const std::vector<std::vector<float>>& matriz, float inicio, float passo) {
    int linhas = matriz.size();
    int colunas = matriz[0].size();

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

void* graphicsThreadFunction(void* arg) {
    if (!glfwInit()) {
        return NULL;
    }

    int width = 600;
    int height = 600;
    float scaleFactor = 0.08f;

    GLFWwindow* window = glfwCreateWindow(width, height, "Mapping 1.0", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return NULL;
    }

    glfwMakeContextCurrent(window);
    glClearColor(1, 1, 1, 1); // fundo branco

    while (!glfwWindowShouldClose(window)) {
        // Atualiza o caminho do robo para desenho
        Position posRobo = {roboPosicao.x * scaleFactor - 0.8, 
                             roboPosicao.y * scaleFactor - 0.7,
                             roboPosicao.theta
        };
        caminho.push_back(posRobo);

        // Renderização
        glClear(GL_COLOR_BUFFER_BIT);

        desenhaGrade(grid.inicio, grid.fim, grid.passo);
        pintaCelulas(matrizMundo, grid.inicio, grid.passo);

        // Desenha caminho
        glColor3f(1.0f, 0.0f, 0.0f);
        glBegin(GL_LINE_STRIP);
        for (auto& pos : caminho) {
            glVertex2f(pos.x, pos.y);
        }
        glEnd();

        // Desenha sensores
        if (!sonares.empty()) {
            glLineWidth(1.5f);
            for (int i = 0; i <= 15; i++) {
                float sensorLength = std::min(sonares[i], 2.0f) * scaleFactor;
                float xFinal = posRobo.x + cos(posRobo.theta - sensorAngles[i] * M_PI / 180) * sensorLength;
                float yFinal = posRobo.y + sin(posRobo.theta - sensorAngles[i] * M_PI / 180) * sensorLength;

                glBegin(GL_LINES);
                if (i == 0 || i == 7 || i == 8 || i == 15)
                    glColor3f(0.9f, 0.1f, 0.9f);
                else
                    glColor3f(0.9f, 0.7f, 0.1f);

                glVertex2f(posRobo.x, posRobo.y);
                glVertex2f(xFinal, yFinal);
                glEnd();
            }
        }

        // Desenha o robo (círculo)
        glColor3f(0.0f, 0.0f, 0.8f);
        float tamanho = 0.02f;
        int numSegmentos = 30;

        glBegin(GL_TRIANGLE_FAN);
            glVertex2f(posRobo.x, posRobo.y);
            for (int i = 0; i <= numSegmentos; i++) {
                float angulo = 2.0f * M_PI * i / numSegmentos;
                float x = posRobo.x + cos(angulo) * tamanho;
                float y = posRobo.y + sin(angulo) * tamanho;
                glVertex2f(x, y);
            }
        glEnd();

        // Linha de direção
        float comprimentoLinha = 0.1f;
        float xFinal = posRobo.x + cos(posRobo.theta) * comprimentoLinha;
        float yFinal = posRobo.y + sin(posRobo.theta) * comprimentoLinha;

        glColor3f(0.1f, 0.6f, 0.2f);
        glLineWidth(3.0f);
        glBegin(GL_LINES);
            glVertex2f(posRobo.x, posRobo.y);
            glVertex2f(xFinal, yFinal);
        glEnd();

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return NULL;
}
