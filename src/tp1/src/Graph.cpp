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
const std::vector<float> offset = {0.0, 0.0};
const float scaleFactor = 0.03f;
const std::vector<double> sensorAngles = {-90, -50, -30, -10, 10, 30, 50, 90, 90, 130, 150, 170, -170, -150, -130, -90};


// Histórico de posições
extern std::vector<Position> caminho;

// Cria Grade e Matrizes
extern GridInfo grid;
extern int size;  
extern std::vector<std::vector<float>> matrizMundo;
extern std::vector<std::vector<int>> matrizPath;
extern std::vector<std::vector<bool>> knownRegion;


void desenhaGrade(float inicio, float fim, float passo) {
    glColor3f(0.85f, 0.0f, 0.0f);
    glLineWidth(0.5f);
    glBegin(GL_LINES);

    int numLinhas = static_cast<int>((fim - inicio) / passo) + 1;

    // Linhas verticais
    for (int i = 0; i < numLinhas; ++i) {
        float x = inicio + i * passo;
        glVertex2f(x, inicio);
        glVertex2f(x, fim);
    }

    // Linhas horizontais
    for (int i = 0; i < numLinhas; ++i) {
        float y = inicio + i * passo;
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
            if(matVal >= 1){
                matVal = matVal/15.0f;
            }
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

void desenhaCaminho(const std::vector<Position>& caminho) {
    glColor3f(1.0f, 0.0f, 0.0f);
    glBegin(GL_LINE_STRIP);
    for (const auto& pos : caminho) {
        glVertex2f(pos.x, pos.y);
    }
    glEnd();
}

void desenhaSensores(const Position& posRobo) {
    if (sonares.empty()) return;

    glLineWidth(1.0f);
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

void desenhaRobo(const Position& posRobo) {
    glColor3f(0.0f, 0.0f, 0.8f);
    float tamanho = 0.01f;
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
}

void desenhaDirecao(const Position& posRobo) {
    float comprimentoLinha = 0.05f;
    float xFinal = posRobo.x + cos(posRobo.theta) * comprimentoLinha;
    float yFinal = posRobo.y + sin(posRobo.theta) * comprimentoLinha;

    glColor3f(0.1f, 0.6f, 0.2f);
    glLineWidth(1.0f);
    glBegin(GL_LINES);
    glVertex2f(posRobo.x, posRobo.y);
    glVertex2f(xFinal, yFinal);
    glEnd();
}

void desenhaKnownRegion(GLFWwindow* windowKnown) {
    glfwMakeContextCurrent(windowKnown);
    glViewport(0, 0, 200, 200);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(grid.inicio, grid.fim, grid.inicio, grid.fim, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glClearColor(1.0, 1.0, 1.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);

    for (size_t i = 0; i < knownRegion.size(); ++i) {
        for (size_t j = 0; j < knownRegion[i].size(); ++j) {
            if (knownRegion[i][j]) {
                float x = grid.inicio + j * grid.passo;
                float y = grid.inicio + i * grid.passo;

                glColor3f(0.0f, 0.0f, 0.0f);
                glBegin(GL_QUADS);
                glVertex2f(x, y);
                glVertex2f(x + grid.passo, y);
                glVertex2f(x + grid.passo, y + grid.passo);
                glVertex2f(x, y + grid.passo);
                glEnd();
            }
        }
    }

    glfwSwapBuffers(windowKnown);
}

void* graphicsThreadFunction(void* arg) {
    if (!glfwInit()) return NULL;

    int width = 600, height = 600;

    GLFWwindow* window = glfwCreateWindow(width, height, "Mapping 1.0", NULL, NULL);
    GLFWwindow* windowKnown = glfwCreateWindow(200, 200, "Known Region", NULL, NULL);
    if (!window || !windowKnown) {
        glfwTerminate();
        return NULL;
    }

    glfwMakeContextCurrent(window);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(grid.inicio, grid.fim, grid.inicio, grid.fim, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glClearColor(1, 1, 1, 1);

    while (!glfwWindowShouldClose(window) && !glfwWindowShouldClose(windowKnown)) {
        Position posRobo = {
            roboPosicao.x * scaleFactor - offset[0], 
            roboPosicao.y * scaleFactor - offset[1],
            roboPosicao.theta
        };
        caminho.push_back(posRobo);

        // Primeira janela
        glfwMakeContextCurrent(window);
        glClear(GL_COLOR_BUFFER_BIT);
        pintaCelulas(matrizMundo, grid.inicio, grid.passo);
        desenhaCaminho(caminho);
        desenhaSensores(posRobo);
        desenhaRobo(posRobo);
        desenhaDirecao(posRobo);
        glfwSwapBuffers(window);

        // Segunda janela
        desenhaKnownRegion(windowKnown);

        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwDestroyWindow(windowKnown);
    glfwTerminate();
    return NULL;
}