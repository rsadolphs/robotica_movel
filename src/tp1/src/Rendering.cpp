#include "Rendering.hpp"
#include "Mapping.hpp"
#include "Explorer.hpp"
#include "Potential.hpp"
#include <GLFW/glfw3.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <mutex>
#include <unistd.h>
#include <set>
#include <atomic>

extern Position robotPosition;
extern std::atomic<char> pressedKey;


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

static void drawDirectionalBiasOverlay(float robotWorldX, float robotWorldY, float robotTheta, float visionRadius, float bias, int segments = 40)
{
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    auto drawSector = [&](float startAngle, float endAngle, float r, float g, float b, float a) {
        glColor4f(r, g, b, a);
        glBegin(GL_TRIANGLE_FAN);
        glVertex2f(robotWorldX, robotWorldY);
        for (int i = 0; i <= segments; ++i)
        {
            float t = static_cast<float>(i) / static_cast<float>(segments);
            float angle = startAngle + t * (endAngle - startAngle);
            float x = robotWorldX + visionRadius * std::cos(angle);
            float y = robotWorldY + visionRadius * std::sin(angle);
            glVertex2f(x, y);
        }
        glEnd();
    };

    const bool rightPreferred = bias > 0.0f;
    const bool leftPreferred = bias < 0.0f;
    const float alphaPreferred = 0.22f;
    const float alphaNonPreferred = 0.12f;

    float preferredColorR = rightPreferred ? 0.0f : 0.0f;
    float preferredColorG = rightPreferred ? 0.8f : 0.8f;
    float preferredColorB = rightPreferred ? 0.0f : 0.0f;
    float nonPreferredColorR = rightPreferred ? 1.0f : 0.8f;
    float nonPreferredColorG = rightPreferred ? 0.3f : 0.2f;
    float nonPreferredColorB = rightPreferred ? 0.3f : 0.0f;

    if (bias > 0.0f)
    {
        drawSector(robotTheta, robotTheta - M_PI_2, preferredColorR, preferredColorG, preferredColorB, alphaPreferred);
        drawSector(robotTheta, robotTheta + M_PI_2, nonPreferredColorR, nonPreferredColorG, nonPreferredColorB, alphaNonPreferred);
    }
    else if (bias < 0.0f)
    {
        drawSector(robotTheta, robotTheta + M_PI_2, preferredColorR, preferredColorG, preferredColorB, alphaPreferred);
        drawSector(robotTheta, robotTheta - M_PI_2, nonPreferredColorR, nonPreferredColorG, nonPreferredColorB, alphaNonPreferred);
    }
    else
    {
        drawSector(robotTheta, robotTheta + M_PI_2, 0.5f, 0.5f, 0.5f, 0.12f);
        drawSector(robotTheta, robotTheta - M_PI_2, 0.5f, 0.5f, 0.5f, 0.12f);
    }

    glColor4f(0.0f, 0.0f, 0.0f, 0.35f);
    glLineWidth(1.0f);
    glBegin(GL_LINE_STRIP);
    for (int i = 0; i <= segments; ++i)
    {
        float angle = robotTheta + i * (M_PI_2 / segments);
        float x = robotWorldX + visionRadius * std::cos(angle);
        float y = robotWorldY + visionRadius * std::sin(angle);
        glVertex2f(x, y);
    }
    glEnd();
    glBegin(GL_LINE_STRIP);
    for (int i = 0; i <= segments; ++i)
    {
        float angle = robotTheta - i * (M_PI_2 / segments);
        float x = robotWorldX + visionRadius * std::cos(angle);
        float y = robotWorldY + visionRadius * std::sin(angle);
        glVertex2f(x, y);
    }
    glEnd();
    glDisable(GL_BLEND);
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

    static const unsigned char letters[][5] = {
        {0b010,0b101,0b111,0b101,0b101}, // A
        {0b110,0b101,0b110,0b101,0b110}, // B (not used)
        {0b111,0b100,0b100,0b100,0b111}, // C
        {0b110,0b101,0b101,0b101,0b110}, // D (not used)
        {0b111,0b100,0b110,0b100,0b111}, // E
        {0b111,0b100,0b110,0b100,0b100}, // F (not used)
        {0b111,0b100,0b101,0b101,0b111}, // G (not used)
        {0b101,0b101,0b111,0b101,0b101}, // H (not used)
        {0b111,0b010,0b010,0b010,0b111}, // I
        {0b001,0b001,0b001,0b101,0b010}, // J (not used)
        {0b101,0b101,0b110,0b101,0b101}, // K (not used)
        {0b100,0b100,0b100,0b100,0b111}, // L
        {0b101,0b111,0b111,0b101,0b101}, // M
        {0b101,0b111,0b111,0b111,0b101}, // N
        {0b111,0b101,0b101,0b101,0b111}, // O
        {0b110,0b101,0b110,0b100,0b100}, // P
        {0b111,0b101,0b101,0b111,0b001}, // Q (not used)
        {0b110,0b101,0b110,0b101,0b101}, // R
        {0b111,0b100,0b111,0b001,0b111}, // S
        {0b111,0b010,0b010,0b010,0b010}, // T
        {0b101,0b101,0b101,0b101,0b111}, // U (not used)
        {0b101,0b101,0b101,0b101,0b010}, // V (not used)
        {0b101,0b101,0b111,0b111,0b101}, // W (not used)
        {0b101,0b101,0b010,0b101,0b101}, // X
        {0b101,0b101,0b010,0b010,0b010}, // Y (not used)
        {0b111,0b001,0b010,0b100,0b111}, // Z (not used)
    };

    static const unsigned char colon[5] = {0b000,0b010,0b000,0b010,0b000};

    const unsigned char* glyph = nullptr;
    unsigned char local[5] = {0,0,0,0,0};

    if (c >= '0' && c <= '9') {
        glyph = font3x5[c - '0'];
    } else if (c >= 'A' && c <= 'Z') {
        int idx = c - 'A';
        if (idx >= 0 && idx < static_cast<int>(sizeof(letters) / 5)) {
            glyph = letters[idx];
        }
    } else if (c == ':') {
        glyph = colon;
    } else if (c == ' ') {
        glyph = local;
    } else {
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

struct Button {
    float x, y, width, height;
    std::string label;
    char actionKey;
    bool active;

    bool contains(float px, float py) const {
        return px >= x && px <= x + width && py >= y && py <= y + height;
    }
};

static bool exploreModeActive = false;
static bool radarModeActive = false;
static bool explorationTimerActive = false;
static double explorationTimerStart = 0.0;
static double explorationTimerLast = 0.0;

static Button makeControlButton(int width, int height, float xFactor, char actionKey, bool active, const std::string& label = "") {
    float buttonSize = height * 0.65f;
    float buttonY = (height - buttonSize) * 0.5f;
    float buttonX = width * xFactor;
    return { buttonX, buttonY, buttonSize, buttonSize, label, actionKey, active };
}

static void drawButton(const Button &button) {
    bool isStartButton = (button.actionKey == 'T');
    bool isRadarButton = (button.actionKey == '3');
    bool isStart = !button.active; // show play when inactive, pause when active

    float baseR, baseG, baseB;
    if (isStartButton) {
        if (isStart) { baseR = 0.05f; baseG = 0.55f; baseB = 0.15f; }
        else { baseR = 0.55f; baseG = 0.05f; baseB = 0.05f; }
    } else if (isRadarButton) {
        if (button.active) { baseR = 0.05f; baseG = 0.55f; baseB = 0.15f; }
        else { baseR = 0.20f; baseG = 0.20f; baseB = 0.20f; }
    } else {
        baseR = isStart ? 0.05f : 0.4f;
        baseG = isStart ? 0.55f : 0.05f;
        baseB = isStart ? 0.15f : 0.15f;
    }

    float highlight = button.active ? 0.18f : 0.0f;
    glColor3f(baseR + highlight, baseG + highlight, baseB + highlight);
    glBegin(GL_QUADS);
        glVertex2f(button.x, button.y);
        glVertex2f(button.x + button.width, button.y);
        glVertex2f(button.x + button.width, button.y + button.height);
        glVertex2f(button.x, button.y + button.height);
    glEnd();

    glColor3f(1.0f, 1.0f, 1.0f);
    glLineWidth(2.0f);
    glBegin(GL_LINE_LOOP);
        glVertex2f(button.x, button.y);
        glVertex2f(button.x + button.width, button.y);
        glVertex2f(button.x + button.width, button.y + button.height);
        glVertex2f(button.x, button.y + button.height);
    glEnd();
    glLineWidth(1.0f);

    float cx = button.x + button.width * 0.5f;
    float cy = button.y + button.height * 0.5f;
    if (!button.label.empty()) {
        float labelSize = std::max(8.0f, button.width * 0.22f);
        drawString(cx - button.width * 0.20f, cy - button.height * 0.10f, labelSize, button.label, 1.0f, 1.0f, 1.0f);
    } else if (isStart) {
        float half = button.width * 0.18f;
        glBegin(GL_TRIANGLES);
            glVertex2f(cx - half * 0.8f, cy - half * 1.2f);
            glVertex2f(cx - half * 0.8f, cy + half * 1.2f);
            glVertex2f(cx + half * 1.2f, cy);
        glEnd();
    } else {
        float barW = button.width * 0.14f;
        float barH = button.height * 0.55f;
        float gap = button.width * 0.10f;
        float leftX = cx - gap * 0.5f - barW;
        float rightX = cx + gap * 0.5f;
        float topY = cy + barH * 0.5f;
        float botY = cy - barH * 0.5f;
        glBegin(GL_QUADS);
            glVertex2f(leftX, botY);
            glVertex2f(leftX + barW, botY);
            glVertex2f(leftX + barW, topY);
            glVertex2f(leftX, topY);
            glVertex2f(rightX, botY);
            glVertex2f(rightX + barW, botY);
            glVertex2f(rightX + barW, topY);
            glVertex2f(rightX, topY);
        glEnd();
    }
}

static void drawControlPanel(int width, int height) {
    glColor3f(0.10f, 0.10f, 0.14f);
    glBegin(GL_QUADS);
        glVertex2f(0.0f, 0.0f);
        glVertex2f(width, 0.0f);
        glVertex2f(width, height);
        glVertex2f(0.0f, height);
    glEnd();

    // control buttons
    Button toggleButton = makeControlButton(width, height, 0.08f, 'T', exploreModeActive);
    Button radarButton = makeControlButton(width, height, 0.20f, '3', radarModeActive);
    drawButton(toggleButton);
    drawButton(radarButton);

    double elapsed = explorationTimerActive ? (glfwGetTime() - explorationTimerStart) : explorationTimerLast;
    int totalSeconds = static_cast<int>(elapsed + 0.5);
    std::string timerText = (totalSeconds < 10 ? "0" : "") + std::to_string(totalSeconds);

    float fontSize = 12.0f;
    float textWidth = timerText.size() * fontSize * 4.0f;
    float textX = width - textWidth - width * 0.08f;
    float textY = height * 0.60f;

    glColor3f(0.16f, 0.16f, 0.24f);
    glBegin(GL_QUADS);
        glVertex2f(textX - 10.0f, textY - 6.0f);
        glVertex2f(textX + textWidth + 10.0f, textY - 6.0f);
        glVertex2f(textX + textWidth + 10.0f, textY + 30.0f);
        glVertex2f(textX - 10.0f, textY + 30.0f);
    glEnd();

    drawString(textX, textY + 20.0f, fontSize, timerText, 1.0f, 1.0f, 1.0f);
}

static void drawPotentialField()
{
    Potential::FieldState fieldState = Potential::getLatestFieldState();
    if (!fieldState.valid)
    {
        // fallback background when field not available
        auto cells = getVisitedCells();
        GridInfo grid = calculateGridSize(cells);
        glColor3f(0.10f, 0.10f, 0.12f);
        glBegin(GL_QUADS);
            glVertex2f(grid.inicio, grid.inicio);
            glVertex2f(grid.fim, grid.inicio);
            glVertex2f(grid.fim, grid.fim);
            glVertex2f(grid.inicio, grid.fim);
        glEnd();
        // subtle grid lines
        // glColor3f(0.18f, 0.18f, 0.18f);
        // glBegin(GL_LINES);
        // for (float i = grid.inicio; i <= grid.fim; i += grid.passo) {
        //     glVertex2f(i, grid.inicio);
        //     glVertex2f(i, grid.fim);
        //     glVertex2f(grid.inicio, i);
        //     glVertex2f(grid.fim, i);
        // }
        // glEnd();
        return;
    }

    auto index = [&](int x, int y) {
        return (y - fieldState.minY) * fieldState.width + (x - fieldState.minX);
    };

    for (int y = fieldState.minY; y <= fieldState.maxY; ++y)
    {
        for (int x = fieldState.minX; x <= fieldState.maxX; ++x)
        {
            int idx = index(x, y);
            if (!fieldState.active[idx])
                continue;

            float value = fieldState.values[idx];
            float clamped = std::min(1.0f, std::max(0.0f, value));
            float r = clamped;
            float g = 0.0f;
            float b = 1.0f - clamped;

            glColor3f(r, g, b);
            glBegin(GL_QUADS);
                glVertex2f(x, y);
                glVertex2f(x + 1.0f, y);
                glVertex2f(x + 1.0f, y + 1.0f);
                glVertex2f(x, y + 1.0f);
            glEnd();
        }
    }

    // glColor3f(0.2f, 0.2f, 0.2f);
    // glBegin(GL_LINES);
    // for (int x = fieldState.minX; x <= fieldState.maxX; ++x) {
    //     glVertex2f(x, fieldState.minY);
    //     glVertex2f(x, fieldState.maxY + 1.0f);
    // }
    // for (int y = fieldState.minY; y <= fieldState.maxY; ++y) {
    //     glVertex2f(fieldState.minX, y);
    //     glVertex2f(fieldState.maxX + 1.0f, y);
    // }
    // glEnd();
}

static void updateExplorationTimer(bool active) {
    if (active && !explorationTimerActive) {
        // starting or resuming: keep previously accumulated time
        explorationTimerActive = true;
        explorationTimerStart = glfwGetTime() - explorationTimerLast;
    }
    if (!active && explorationTimerActive) {
        // pausing: store accumulated elapsed time
        explorationTimerActive = false;
        explorationTimerLast = glfwGetTime() - explorationTimerStart;
    }
}

static void setExplorationActive(bool active) {
    if (active != explorationTimerActive) {
        updateExplorationTimer(active);
    }
}

static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    (void)mods;
    if (button != GLFW_MOUSE_BUTTON_LEFT || action != GLFW_PRESS) return;

    int width, height;
    glfwGetWindowSize(window, &width, &height);
    double xPos, yPos;
    glfwGetCursorPos(window, &xPos, &yPos);
    float mx = static_cast<float>(xPos);
    float my = static_cast<float>(height - yPos);

    float controlHeight = std::max(80, height / 8);
    Button toggleButton = makeControlButton(width, controlHeight, 0.08f, 'T', false);
    Button radarButton = makeControlButton(width, controlHeight, 0.20f, '3', false, "RAD");
    if (toggleButton.contains(mx, my)) {
        if (!exploreModeActive) {
            // start exploration and stop radar if it was active
            pressedKey = '2';
            exploreModeActive = true;
            if (radarModeActive) {
                radarModeActive = false;
            }
            setExplorationActive(true);
        } else {
            // stop exploration
            pressedKey = ' ';
            exploreModeActive = false;
            setExplorationActive(false);
        }
    } else if (radarButton.contains(mx, my)) {
        if (!radarModeActive) {
            // start radar mode and stop exploration if it was active
            pressedKey = '3';
            radarModeActive = true;
            if (exploreModeActive) {
                exploreModeActive = false;
            }
            setExplorationActive(true);
        } else {
            pressedKey = ' ';
            radarModeActive = false;
            setExplorationActive(false);
        }
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


static void drawRobotTriangleInMapPanel(
    float robotWorldX,
    float robotWorldY,
    float robotTheta,
    float viewMinX,
    float viewMaxX,
    float viewMinY,
    float viewMaxY,
    float viewportWidth,
    float viewportHeight,
    float triangleSizePx)
{
    float worldWidth = viewMaxX - viewMinX;
    float worldHeight = viewMaxY - viewMinY;
    if (worldWidth <= 0.0f || worldHeight <= 0.0f) {
        return;
    }

    float screenX = ((robotWorldX - viewMinX) / worldWidth) * viewportWidth;
    float screenY = ((robotWorldY - viewMinY) / worldHeight) * viewportHeight;

    float fx = std::cos(robotTheta);
    float fy = std::sin(robotTheta);
    float px = -fy;
    float py = fx;

    float size = triangleSizePx;
    float halfBack = size * 0.5f;
    float halfWidth = size * 0.35f;

    float tipX = screenX + fx * size;
    float tipY = screenY + fy * size;
    float baseCenterX = screenX - fx * halfBack;
    float baseCenterY = screenY - fy * halfBack;
    float base1X = baseCenterX + px * halfWidth;
    float base1Y = baseCenterY + py * halfWidth;
    float base2X = baseCenterX - px * halfWidth;
    float base2Y = baseCenterY - py * halfWidth;

    glColor3f(0.0f, 0.4f, 0.0f);
    glBegin(GL_TRIANGLES);
        glVertex2f(tipX, tipY);
        glVertex2f(base1X, base1Y);
        glVertex2f(base2X, base2Y);
    glEnd();
}

void* renderingThreadFunction(void* arg) {
    (void)arg;
    if (!glfwInit()) return NULL;

    int width = 1200, height = 900;

    GLFWwindow* window = glfwCreateWindow(width, height, "Mapping", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return NULL;
    }

    glfwMakeContextCurrent(window);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);

    glMatrixMode(GL_MODELVIEW);
    glClearColor(1, 1, 1, 1);

    GridInfo currentGrid;
    bool firstFrame = true;

    float radarRadius = 10.0f;  // Raio de 10 metros no radar

    while (!glfwWindowShouldClose(window)) {

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

        int fbWidth, fbHeight;
        glfwGetFramebufferSize(window, &fbWidth, &fbHeight);
        int controlHeight = std::max(80, fbHeight / 8);
        int mapHeight = fbHeight - controlHeight;
        int rightWidth = fbWidth / 3;
        int leftWidth = fbWidth - rightWidth;
        int halfHeight = mapHeight / 2;

        glEnable(GL_SCISSOR_TEST);

        // Map panel (left)
        glViewport(0, controlHeight, leftWidth, mapHeight);
        glScissor(0, controlHeight, leftWidth, mapHeight);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(newGrid.inicio, newGrid.fim,
                newGrid.inicio, newGrid.fim,
                -1.0, 1.0);
        glMatrixMode(GL_MODELVIEW);

        glClearColor(0.7f, 0.7f, 0.7f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        glLoadIdentity();

        // desenha grid
        // glColor3f(0.0f, 1.0f, 1.0f);  // cyan
        // glBegin(GL_LINES);
        // for (float i = currentGrid.inicio; i <= currentGrid.fim; i += currentGrid.passo) {
        //     glVertex2f(i, currentGrid.inicio);
        //     glVertex2f(i, currentGrid.fim);
        //     glVertex2f(currentGrid.inicio, i);
        //     glVertex2f(currentGrid.fim, i);
        // }
        // glEnd();

        for (const auto& cell : cells)
        {
            drawCell(cell);
        }

        // desenha rastro do robô
        std::vector<Position> path = getRobotPath();
        if (path.size() > 1)
        {
            glColor3f(1.0f, 0.0f, 0.0f);
            glLineWidth(1.0f);
            glBegin(GL_LINE_STRIP);
            for (const auto& pose : path)
            {
                float px = pose.x * 100.0f / cellSizeCentimeters;
                float py = pose.y * 100.0f / cellSizeCentimeters;
                glVertex2f(px, py);
            }
            glEnd();
            glLineWidth(1.0f);
        }

        float robotCellX = robotPosition.x * 100.0f / 10.0f;
        float robotCellY = robotPosition.y * 100.0f / 10.0f;
        float theta = robotPosition.theta;
        float visionRadius = Potential::getVisionRadius();
        float bias = Potential::getDirectionalBias();

        drawDirectionalBiasOverlay(robotCellX, robotCellY, theta, visionRadius, bias);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(0.0f, static_cast<float>(leftWidth), 0.0f, static_cast<float>(mapHeight), -1.0f, 1.0f);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        drawRobotTriangleInMapPanel(
            robotCellX,
            robotCellY,
            theta,
            newGrid.inicio,
            newGrid.fim,
            newGrid.inicio,
            newGrid.fim,
            static_cast<float>(leftWidth),
            static_cast<float>(mapHeight),
            16.0f);

        // Radar panel (top-right)
        glViewport(leftWidth, controlHeight + halfHeight, rightWidth, halfHeight);
        glScissor(leftWidth, controlHeight + halfHeight, rightWidth, halfHeight);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(-15.0f, 15.0f, -15.0f, 15.0f, -1.0f, 1.0f);
        glMatrixMode(GL_MODELVIEW);

        glClearColor(0.15f, 0.15f, 0.15f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        glLoadIdentity();

        drawRadar(0.0f, 0.0f, radarRadius);

        // Potential field panel (middle-right)
        glViewport(leftWidth, controlHeight, rightWidth, halfHeight);
        glScissor(leftWidth, controlHeight, rightWidth, halfHeight);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(newGrid.inicio, newGrid.fim,
                newGrid.inicio, newGrid.fim,
                -1.0, 1.0);
        glMatrixMode(GL_MODELVIEW);

        glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        glLoadIdentity();

        drawPotentialField();

        // Control strip (bottom)
        glViewport(0, 0, fbWidth, controlHeight);
        glScissor(0, 0, fbWidth, controlHeight);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(0.0f, static_cast<float>(fbWidth), 0.0f, static_cast<float>(controlHeight), -1.0f, 1.0f);
        glMatrixMode(GL_MODELVIEW);

        glClearColor(0.08f, 0.08f, 0.10f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        glLoadIdentity();

        drawControlPanel(fbWidth, controlHeight);
        glDisable(GL_SCISSOR_TEST);
        
        glfwSwapBuffers(window);
        glfwPollEvents();
        
        usleep(16666);  // ~60 Hz
    }
    
    saveHistoryToFile("mapping_history.txt");
    glfwDestroyWindow(window);
    glfwTerminate();
    return NULL;
}
