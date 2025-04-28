// graphics.cpp
#include "graphics.hpp"
#include <GLFW/glfw3.h>
#include <vector>

// Variável global ou extern para compartilhar posição do robô
extern Position roboPosicao;

// Histórico de posições
std::vector<Position> caminho;

void* graphicsThreadFunction(void* arg) {
    if (!glfwInit()) {
        return NULL;
    }

    GLFWwindow* window = glfwCreateWindow(1000, 800, "Caminho do Robo", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return NULL;
    }

    glfwMakeContextCurrent(window);
    glClearColor(1, 1, 1, 1); // fundo branco

    while (!glfwWindowShouldClose(window)) {
        // Atualiza o caminho
        caminho.push_back(roboPosicao);  // ATENÇÃO: precisa garantir que acesso à roboPosicao é seguro!

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
        glColor3f(0.0f, 0.0f, 1.0f); // azul

        float tamanho = 0.02f;  // metade do lado do quadrado

        // Desenhar o quadrado (corpo do robô)
        glBegin(GL_QUADS);
            glVertex2f(roboPosicao.x - tamanho, roboPosicao.y - tamanho);
            glVertex2f(roboPosicao.x + tamanho, roboPosicao.y - tamanho);
            glVertex2f(roboPosicao.x + tamanho, roboPosicao.y + tamanho);
            glVertex2f(roboPosicao.x - tamanho, roboPosicao.y + tamanho);
        glEnd();

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return NULL;
}
