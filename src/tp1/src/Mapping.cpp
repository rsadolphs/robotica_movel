#include "Mapping.hpp"
#include "rclcpp/rclcpp.hpp"

#include <vector>
#include <cmath>
#include <mutex>
#include <unistd.h>
#include <fstream>
#include <string>

// ==========================
// External variables
// ==========================

extern Position robotPosition;     // x, y, theta (em metros / rad)
extern std::vector<float> lasers;  // leituras do laser (ranges)

// ==========================
// Internal variables and configs
// ==========================

int cellSizeCentimeters = 10;

std::vector<Cell> visitedCells;
std::mutex visitedCellsMutex;

static constexpr float POSE_EPS = 1e-3f;
static std::vector<History> history;

// ==========================
// Supporting methods
// ==========================

Cell findCell(float x, float y) {
    /*
        Converts position (meters) to grid cell (integer coordinates)
    */
    Cell currentCell;
    currentCell.x = static_cast<int>(x * 100.0f / cellSizeCentimeters);
    currentCell.y = static_cast<int>(y * 100.0f / cellSizeCentimeters);
    return currentCell;
}

bool cellExists(const Cell& c) {
    for (const auto& cell : visitedCells) {
        if (cell.x == c.x && cell.y == c.y) {
            return true;
        }
    }
    return false;
}

void addToVisited(const Cell& cell) {
    if (!cellExists(cell)) {
        visitedCells.push_back(cell);
    }
}

static bool samePose(const Position& a, const Position& b) {
    return std::abs(a.x - b.x) < POSE_EPS &&
           std::abs(a.y - b.y) < POSE_EPS &&
           std::abs(a.theta - b.theta) < POSE_EPS;
}

void saveHistoryToFile(const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) return;

    for (const auto& h : history) {
        // pose
        file << h.pose.x << " "
             << h.pose.y << " "
             << h.pose.theta;

        // lasers
        for (float r : h.laserReadings) {
            file << " " << r;
        }

        file << "\n";
    }

    file.close();
}

// ==========================
// Bresenham (Cell → Cell)
// ==========================

std::vector<Cell> bresenham(const Cell& start, const Cell& end) {
    std::vector<Cell> cells;

    int x0 = start.x;
    int y0 = start.y;
    int x1 = end.x;
    int y1 = end.y;

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);

    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;

    int err = dx - dy;

    while (true) {
        cells.push_back({x0, y0, {}});

        if (x0 == x1 && y0 == y1) break;

        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }

    return cells;
}

// ==========================
// Laser-based mapping
// ==========================

void updateCellsFromLaser(
    float maxRange = 2.0f   // metros
) {
    std::lock_guard<std::mutex> lock(visitedCellsMutex);

    const float fov = M_PI;  // 180 graus
    const float angleStep = fov / static_cast<float>(lasers.size());
    const float startAngle = -fov / 2.0f;

    Cell robotCell = findCell(robotPosition.x, robotPosition.y);

    for (size_t i = 0; i < lasers.size(); ++i) {

        float range = lasers[i];
        bool noDetect = false;

        if (range > maxRange) {
            range = maxRange;
            noDetect = true;
        }

        float angle = robotPosition.theta + startAngle
            + (lasers.size() - 1 - i) * angleStep;


        float xEnd = robotPosition.x + std::cos(angle) * range;
        float yEnd = robotPosition.y + std::sin(angle) * range;

        Cell endCell = findCell(xEnd, yEnd);

        std::vector<Cell> ray = bresenham(robotCell, endCell);

        // células livres (todas menos a última)
        for (size_t k = 0; k + 1 < ray.size(); ++k) {
            Cell freeCell = ray[k];
            freeCell.properties.isFree = true;
            freeCell.properties.isOccupied = false;
            addToVisited(freeCell);
        }

        // célula ocupada (se houve detecção)
        if (!ray.empty() && !noDetect) {
            Cell occCell = ray.back();
            occCell.properties.isFree = false;
            occCell.properties.isOccupied = true;
            addToVisited(occCell);
        }
    }
}

// ==========================
// Entrypoint
// ==========================

void* mappingThreadFunction(void* arg) {

    while (rclcpp::ok()) {

        // adiciona célula atual do robô
        {
            std::lock_guard<std::mutex> lock(visitedCellsMutex);
            Cell currentCell = findCell(robotPosition.x, robotPosition.y);
            currentCell.properties.isFree = true;
            addToVisited(currentCell);
        }

            // SALVA HISTÓRICO
            if (history.empty() ||
                !samePose(history.back().pose, robotPosition)) {

                history.push_back({robotPosition, lasers});

                if (history.size() > 10000) {
                    history.erase(history.begin());
                }
            }

        // atualiza mapa a partir do laser
        updateCellsFromLaser();

        usleep(10000); // 10 ms
    }

    return NULL;
}
