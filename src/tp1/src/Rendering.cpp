#include "Rendering.hpp"
#include "Mapping.hpp"
#include "Explorer.hpp"
#include <GLFW/glfw3.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <mutex>
#include <unistd.h>
#include <set>

extern Position robotPosition;


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


void drawCell(Cell cell) {
    float size = 1.0f;
    int x = cell.x;
    int y = cell.y;
    Color color = {0.7f, 0.7f, 0.7f}; // unknown cell default

   if(cell.properties.isFrontier)
    {
        color = {1.0f, 0.0f, 0.0f};
    }
    else if(cell.properties.isFree)
    {
        color = {1.0f, 1.0f, 1.0f};
    }
    else if(cell.properties.isOccupied)
    {
        color = {0.0f, 0.0f, 0.0f};
    }

    glColor3f(color.r, color.g, color.b);
    glBegin(GL_QUADS);
        glVertex2f(x,     y);
        glVertex2f(x + size, y);
        glVertex2f(x + size, y + size);
        glVertex2f(x,     y + size);
    glEnd();
}

// ======================================================
// RADAR / MINI-MAPA
// ======================================================

void drawCircle(float centerX, float centerY, float radius, int segments = 100) {
    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < segments; ++i) {
        float angle = 2.0f * M_PI * i / segments;
        float x = centerX + radius * std::cos(angle);
        float y = centerY + radius * std::sin(angle);
        glVertex2f(x, y);
    }
    glEnd();
}

void drawPoint(float x, float y, float size, float r, float g, float b) {
    glColor3f(r, g, b);
    glBegin(GL_QUADS);
        glVertex2f(x - size/2, y - size/2);
        glVertex2f(x + size/2, y - size/2);
        glVertex2f(x + size/2, y + size/2);
        glVertex2f(x - size/2, y + size/2);
    glEnd();
}

// Simple 3x5 pixel font for digits and a few symbols ((),.,,)
void drawChar(float x, float y, float pixelSize, char c, float r, float g, float b) {
    static const unsigned char font3x5[][5] = {
        // 0-9
        {0b111,0b101,0b101,0b101,0b111}, //0
        {0b010,0b110,0b010,0b010,0b111}, //1
        {0b111,0b001,0b111,0b100,0b111}, //2
        {0b111,0b001,0b111,0b001,0b111}, //3
        {0b101,0b101,0b111,0b001,0b001}, //4
        {0b111,0b100,0b111,0b001,0b111}, //5
        {0b111,0b100,0b111,0b101,0b111}, //6
        {0b111,0b001,0b010,0b010,0b010}, //7
        {0b111,0b101,0b111,0b101,0b111}, //8
        {0b111,0b101,0b111,0b001,0b111}, //9
    };

    static const unsigned char parenL[5] = {0b010,0b100,0b100,0b100,0b010};
    static const unsigned char parenR[5] = {0b010,0b001,0b001,0b001,0b010};
    static const unsigned char dot[5]    = {0b000,0b000,0b000,0b000,0b010};
    static const unsigned char comma[5]  = {0b000,0b000,0b000,0b000,0b010};

    const unsigned char* glyph = nullptr;
    unsigned char local[5];

    if (c >= '0' && c <= '9') {
        glyph = font3x5[c - '0'];
    } else if (c == '(') {
        glyph = parenL;
    } else if (c == ')') {
        glyph = parenR;
    } else if (c == '.') {
        glyph = dot;
    } else if (c == ',') {
        glyph = comma;
    } else {
        // unknown -> space
        for (int i=0;i<5;i++) local[i]=0;
        glyph = local;
    }

    glColor3f(r,g,b);
    for (int row = 0; row < 5; ++row) {
        unsigned char bits = glyph[row];
        for (int col = 0; col < 3; ++col) {
            if (bits & (1 << (2-col))) {
                float px = x + col * pixelSize;
                float py = y - row * pixelSize;
                glBegin(GL_QUADS);
                    glVertex2f(px, py);
                    glVertex2f(px + pixelSize, py);
                    glVertex2f(px + pixelSize, py + pixelSize);
                    glVertex2f(px, py + pixelSize);
                glEnd();
            }
        }
    }
}

void drawString(float x, float y, float pixelSize, const std::string &s, float r, float g, float b) {
    float cx = x;
    for (char c : s) {
        if (c == ' ') {
            cx += pixelSize * 2.0f;
            continue;
        }
        drawChar(cx, y, pixelSize, c, r, g, b);
        cx += pixelSize * 4.0f; // advance (3 pixels + 1 spacing)
    }
}

struct RadarCoord {
    float x;
    float y;
    bool outOfBounds;  // true if projected to edge (distance > radarRadius)
};

// Converte coordenadas cartesianas globais para coordenadas do radar
// O robô fica no centro (0, 0) do radar
// radarRadius = raio do círculo do radar em metros
// Se a distância for > radarRadius, projeta o ponto na borda mantendo o ângulo
RadarCoord convertToRadarCoords(
    float centroidX, float centroidY,
    float robotX, float robotY,
    float robotTheta,
    float radarRadius)
{
    // Translada o centroide para o sistema de coordenadas do robô
    float relX = centroidX - robotX;
    float relY = centroidY - robotY;
    
    // Rotaciona para o frame do robô (onde o robô aponta para cima)
    float cosTheta = std::cos(robotTheta);
    float sinTheta = std::sin(robotTheta);
    
    float rotatedX = relX * cosTheta + relY * sinTheta;
    float rotatedY = -relX * sinTheta + relY * cosTheta;
    
    // Calcula distância
    float distance = std::sqrt(rotatedX * rotatedX + rotatedY * rotatedY);

    RadarCoord result;
    result.outOfBounds = false;

    // Mapear: rotatedX = avanço (frente), rotatedY = direita
    // Queremos que frente (+rotatedX) aponte para cima do radar (y positivo),
    // e a direita (+rotatedY) aponte para a direita do radar (x positivo).
    if (distance > radarRadius) {
        // projeta para a borda mantendo direção
        float nx = rotatedX / distance; // forward normalized
        float ny = rotatedY / distance; // right normalized
        // invert X so right-hand side in world maps to +X on radar correctly
        result.x = -ny * radarRadius;
        result.y = nx * radarRadius;
        result.outOfBounds = true;
    } else {
        // posição proporcional dentro do raio
        result.x = -rotatedY; // right → x (inverted to correct handedness)
        result.y = rotatedX; // forward → y
        result.outOfBounds = false;
    }
    return result;
}

void drawRadar(float radarCenterX, float radarCenterY, float radarRadius) {
    // Background do radar
    glColor3f(0.2f, 0.2f, 0.3f);
    glBegin(GL_TRIANGLE_FAN);
    for (int i = 0; i <= 100; ++i) {
        float angle = 2.0f * M_PI * i / 100.0f;
        float x = radarCenterX + radarRadius * std::cos(angle);
        float y = radarCenterY + radarRadius * std::sin(angle);
        glVertex2f(x, y);
    }
    glEnd();
    
    // Círculo do radar (borda)
    glColor3f(0.0f, 1.0f, 0.0f);
    drawCircle(radarCenterX, radarCenterY, radarRadius, 100);
    
    // Círculos de referência (distâncias)
    glColor3f(0.3f, 0.5f, 0.3f);
    drawCircle(radarCenterX, radarCenterY, radarRadius * 0.5f, 50);
    drawCircle(radarCenterX, radarCenterY, radarRadius * 0.33f, 40);
    
    // Marcadores de direção (cruz)
    glColor3f(0.0f, 1.0f, 0.0f);
    glBegin(GL_LINES);
        // Vertical (up/forward)
        glVertex2f(radarCenterX, radarCenterY + radarRadius * 0.9f);
        glVertex2f(radarCenterX, radarCenterY + radarRadius);
        // Horizontal (left)
        glVertex2f(radarCenterX - radarRadius * 0.9f, radarCenterY);
        glVertex2f(radarCenterX - radarRadius, radarCenterY);
    glEnd();
    
    // Robô no centro (triângulo fixo apontando para cima)
    glColor3f(1.0f, 1.0f, 0.0f);
    glBegin(GL_TRIANGLES);
        glVertex2f(radarCenterX, radarCenterY + 0.3f);
        glVertex2f(radarCenterX - 0.2f, radarCenterY - 0.2f);
        glVertex2f(radarCenterX + 0.2f, radarCenterY - 0.2f);
    glEnd();
    
    // Desenha todos os clusters e mostra label (tamanho, distância)
    auto clusters = detectFrontierClusters();
    for (size_t i = 0; i < clusters.size(); ++i) {
        const auto& cluster = clusters[i];

        if (std::isnan(cluster.centroidX) || std::isnan(cluster.centroidY) ||
            std::isinf(cluster.centroidX) || std::isinf(cluster.centroidY)) {
            continue;
        }

        float cx_m = cluster.centroidX * (cellSizeCentimeters / 100.0f);
        float cy_m = cluster.centroidY * (cellSizeCentimeters / 100.0f);

        float dx = cx_m - robotPosition.x;
        float dy = cy_m - robotPosition.y;
        float dist = std::sqrt(dx*dx + dy*dy);

        RadarCoord radarCoord = convertToRadarCoords(
            cx_m, cy_m,
            robotPosition.x, robotPosition.y,
            robotPosition.theta,
            radarRadius
        );

        float displayX = radarCenterX + radarCoord.x;
        float displayY = radarCenterY + radarCoord.y;

        int sizeCells = static_cast<int>(cluster.cells.size());

        // color: small clusters (<5) -> white; else inside cyan, outside red
        if (sizeCells < 5) {
            drawPoint(displayX, displayY, 0.3f, 1.0f, 1.0f, 1.0f);
        } else if (radarCoord.outOfBounds) {
            drawPoint(displayX, displayY, 0.3f, 1.0f, 0.0f, 0.0f);
        } else {
            drawPoint(displayX, displayY, 0.25f, 0.0f, 1.0f, 1.0f);
        }

        // label: show only for clusters with size >= 5
        if (sizeCells >= 5) {
            int dist10 = static_cast<int>(dist * 10.0f + 0.5f); // one decimal
            int meters = dist10 / 10;
            int dec = dist10 % 10;

            std::string label = "(" + std::to_string(sizeCells) + ", " + std::to_string(meters) + "." + std::to_string(dec) + ")";

            // draw label with simple collision avoidance to reduce overlaps
            float pixelSize = 0.15f; // radar units

            struct Box { float l, t, r, b; };
            static std::vector<Box> placed; // stores boxes within this radar draw call
            if (i == 0) placed.clear();

            int chars = static_cast<int>(label.size());
            // base candidate offsets (from marker)
            const float base = 0.4f;
            const std::vector<std::pair<float,float>> dirs = {
                { base,  base}, { base, -base}, {-base,  base}, {-base, -base},
                { base*2, 0.0f}, {-base*2, 0.0f}, {0.0f, base*2}, {0.0f, -base*2}
            };

            auto overlaps = [&](const Box &a, const Box &b) {
                // no overlap if one is completely to one side
                if (a.r < b.l) return false;
                if (a.l > b.r) return false;
                if (a.b > b.t) return false;
                if (a.t < b.b) return false;
                return true;
            };

            float chosenX = displayX + base;
            float chosenY = displayY + base;
            bool placedOk = false;

            for (int radius = 0; radius < 4 && !placedOk; ++radius) {
                for (const auto &d : dirs) {
                    float tryX = displayX + d.first * (1.0f + radius);
                    float tryY = displayY + d.second * (1.0f + radius);

                    Box candidate;
                    candidate.l = tryX;
                    candidate.t = tryY;
                    candidate.r = tryX + chars * pixelSize * 4.0f;
                    candidate.b = tryY - 5.0f * pixelSize;

                    bool any = false;
                    for (const auto &pb : placed) {
                        if (overlaps(candidate, pb)) { any = true; break; }
                    }
                    if (!any) {
                        chosenX = tryX;
                        chosenY = tryY;
                        placed.push_back(candidate);
                        placedOk = true;
                        break;
                    }
                }
            }

            if (!placedOk) {
                // fallback: place at default offset and accept overlap
                Box fallback;
                fallback.l = chosenX;
                fallback.t = chosenY;
                fallback.r = chosenX + chars * pixelSize * 4.0f;
                fallback.b = chosenY - 5.0f * pixelSize;
                placed.push_back(fallback);
            }

            drawString(chosenX, chosenY, pixelSize, label, 1.0f, 1.0f, 1.0f);
        }
    }
}


void* renderingThreadFunction(void* arg) {
    if (!glfwInit()) return NULL;

    int width = 600, height = 600;

    GLFWwindow* window = glfwCreateWindow(width, height, "Mapping", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return NULL;
    }

    // Criar segunda janela (radar)
    int radarWidth = 500, radarHeight = 500;
    GLFWwindow* radarWindow = glfwCreateWindow(radarWidth, radarHeight, "Radar / Mini-Map", NULL, window);
    if (!radarWindow) {
        glfwDestroyWindow(window);
        glfwTerminate();
        return NULL;
    }

    glfwMakeContextCurrent(window);

    glMatrixMode(GL_MODELVIEW);
    glClearColor(1, 1, 1, 1);

    GridInfo currentGrid;
    bool firstFrame = true;

    float radarRadius = 10.0f;  // Raio de 10 metros no radar

    while (!glfwWindowShouldClose(window) && !glfwWindowShouldClose(radarWindow)) {

       std::vector<Cell> cells = getVisitedCells();

        GridInfo newGrid = calculateGridSize(cells);

        // ====== RENDERIZAR JANELA PRINCIPAL ======
        glfwMakeContextCurrent(window);

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

        glClearColor(0.7f, 0.7f, 0.7f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        glLoadIdentity();

        // desenha grid
        glColor3f(0.0f, 1.0f, 1.0f);  // cyan
        glBegin(GL_LINES);
        for (float i = currentGrid.inicio; i <= currentGrid.fim; i += currentGrid.passo) {
            glVertex2f(i, currentGrid.inicio);
            glVertex2f(i, currentGrid.fim);
            glVertex2f(currentGrid.inicio, i);
            glVertex2f(currentGrid.fim, i);
        }
        glEnd();

        // desenha células visitadas
        for (const auto& cell : cells)
        {
            drawCell(cell);
        }

        // Desenha o robô no mapa principal (triângulo apontando na yaw)
        // Converte pose do robô (metros) para coordenadas de célula
        // cellSizeCentimeters em Mapping.cpp é 10
        float robotCellX = robotPosition.x * 100.0f / 10.0f;
        float robotCellY = robotPosition.y * 100.0f / 10.0f;

        float theta = robotPosition.theta;

        // Vetor frontal: 0 rad aponta para +X (direita)
        float fx = std::cos(theta);
        float fy = std::sin(theta);

        // Perpendicular (to the right)
        float px = -fy;
        float py = fx;

        // Triplica o tamanho solicitado
        float size = 1.8f; // comprimento do triângulo
        float halfBack = size * 0.5f;
        float halfWidth = size * 0.35f;

        float tipX = robotCellX + fx * size;
        float tipY = robotCellY + fy * size;

        float baseCenterX = robotCellX - fx * halfBack;
        float baseCenterY = robotCellY - fy * halfBack;

        float base1X = baseCenterX + px * halfWidth;
        float base1Y = baseCenterY + py * halfWidth;

        float base2X = baseCenterX - px * halfWidth;
        float base2Y = baseCenterY - py * halfWidth;

        glColor3f(0.0f, 0.4f, 0.0f); // verde escuro
        glBegin(GL_TRIANGLES);
            glVertex2f(tipX, tipY);
            glVertex2f(base1X, base1Y);
            glVertex2f(base2X, base2Y);
        glEnd();

        glfwSwapBuffers(window);

        // ====== RENDERIZAR JANELA RADAR ======
        glfwMakeContextCurrent(radarWindow);
        
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(-15.0f, 15.0f, -15.0f, 15.0f, -1.0f, 1.0f);
        glMatrixMode(GL_MODELVIEW);

        glClearColor(0.15f, 0.15f, 0.15f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glLoadIdentity();

        drawRadar(0.0f, 0.0f, radarRadius);

        glfwSwapBuffers(radarWindow);

        glfwPollEvents();
        
        usleep(50000);  // ~20 Hz
    }
    
    saveHistoryToFile("mapping_history.txt");
    glfwDestroyWindow(window);
    glfwDestroyWindow(radarWindow);
    glfwTerminate();
    return NULL;
}
