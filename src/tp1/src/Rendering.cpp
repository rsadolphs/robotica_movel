#include "Rendering.hpp"
#include "Mapping.hpp"
#include <GLFW/glfw3.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <mutex>

extern std::vector<Cell> visitedCells;
extern std::mutex visitedCellsMutex;

GridInfo calculateGridSize(const std::vector<Cell>& visitedCells) {
    GridInfo grid;

    if (visitedCells.empty()) {
        grid.inicio = -10;
        grid.fim    =  10;
        grid.passo  =  1;
        return grid;
    }

    int minX = visitedCells[0].x;
    int maxX = visitedCells[0].x;
    int minY = visitedCells[0].y;
    int maxY = visitedCells[0].y;

    for (const auto& cell : visitedCells) {
        minX = std::min(minX, cell.x);
        maxX = std::max(maxX, cell.x);
        minY = std::min(minY, cell.y);
        maxY = std::max(maxY, cell.y);
    }

    float margin = 2.0f;

    // usa limites reais
    float min = std::min(minX, minY) - margin;
    float max = std::max(maxX, maxY) + margin;

    grid.inicio = min;
    grid.fim    = max;
    grid.passo  = 1.0f;

    return grid;
}


void drawCell(int x, int y) {
    float size = 1.0f;

    glBegin(GL_QUADS);
        glVertex2f(x,     y);
        glVertex2f(x + size, y);
        glVertex2f(x + size, y + size);
        glVertex2f(x,     y + size);
    glEnd();
}


void* renderingThreadFunction(void* arg) {
    if (!glfwInit()) return NULL;

    int width = 600, height = 600;

    GLFWwindow* window = glfwCreateWindow(width, height, "Mapping", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return NULL;
    }

    glfwMakeContextCurrent(window);

    glMatrixMode(GL_MODELVIEW);
    glClearColor(1, 1, 1, 1);

    GridInfo currentGrid;
    bool firstFrame = true;

    while (!glfwWindowShouldClose(window)) {

        // ---------- REGIÃO CRÍTICA ----------
        std::lock_guard<std::mutex> lock(visitedCellsMutex);

        // recalcula grid dinamicamente
        GridInfo newGrid = calculateGridSize(visitedCells);

        // atualiza projeção apenas se necessário
        if (firstFrame ||
            newGrid.inicio != currentGrid.inicio ||
            newGrid.fim    != currentGrid.fim) {

            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            glOrtho(newGrid.inicio, newGrid.fim,
                    newGrid.inicio, newGrid.fim,
                    -1.0, 1.0);

            glMatrixMode(GL_MODELVIEW);

            currentGrid = newGrid;
            firstFrame = false;
        }

        glClear(GL_COLOR_BUFFER_BIT);
        glLoadIdentity();

        // desenha grid
        glColor3f(0.9f, 0.9f, 0.9f);
        glBegin(GL_LINES);
        for (float i = currentGrid.inicio; i <= currentGrid.fim; i += currentGrid.passo) {
            glVertex2f(i, currentGrid.inicio);
            glVertex2f(i, currentGrid.fim);
            glVertex2f(currentGrid.inicio, i);
            glVertex2f(currentGrid.fim, i);
        }
        glEnd();

        // desenha células visitadas
        glColor3f(0.2f, 0.2f, 0.8f);
        for (const auto& cell : visitedCells) {
            drawCell(cell.x, cell.y);
        }
        // ---------- FIM DA REGIÃO CRÍTICA ----------

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return NULL;
}
