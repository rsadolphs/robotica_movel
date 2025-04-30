// graphics.cpp
#include "graphics.hpp"
#include <GLFW/glfw3.h>
#include <vector>
#include <cmath>

// Variável global ou extern para compartilhar posição do robô
extern Position roboPosicao;
extern Position pontoParede;

// Histórico de posições
std::vector<Position> caminho;
std::vector<Position> paredes;

float round2(float valor) {
    return std::round(valor * 100.0f) / 100.0f;
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

        caminho.push_back(posRobo);  // ATENÇÃO: precisa garantir que acesso à roboPosicao é seguro!
        paredes.push_back(posParede);

        // Renderizar
        glClear(GL_COLOR_BUFFER_BIT);

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
