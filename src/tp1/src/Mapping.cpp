#include "Mapping.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cmath>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <unistd.h>
#include <fstream>

extern Position robotPosition;
extern std::vector<float> lasers;

static constexpr float POSE_EPS = 1e-3f;

int cellSizeCentimeters = 10;

std::mutex visitedCellsMutex;
static std::vector<History> history;

// ======================================================
// HASH MAP PARA CÉLULAS
// ======================================================

struct CellKey {
    int x;
    int y;

    bool operator==(const CellKey& other) const {
        return x == other.x && y == other.y;
    }
};

struct CellKeyHash {
    std::size_t operator()(const CellKey& k) const {
        return std::hash<int>()(k.x) ^
              (std::hash<int>()(k.y) << 1);
    }
};

std::unordered_map<CellKey, Cell, CellKeyHash> grid;
std::unordered_set<CellKey, CellKeyHash> frontierSet;

// ======================================================
// UTIL
// ======================================================

Cell findCell(float x, float y)
{
    Cell c;

    c.x = static_cast<int>(
        std::floor(x * 100.0f / cellSizeCentimeters)
    );

    c.y = static_cast<int>(
        std::floor(y * 100.0f / cellSizeCentimeters)
    );

    return c;
}

static bool samePose(
    const Position& a,
    const Position& b)
{
    return std::abs(a.x - b.x) < POSE_EPS &&
           std::abs(a.y - b.y) < POSE_EPS &&
           std::abs(a.theta - b.theta) < POSE_EPS;
}

// ======================================================
// HIMM
// ======================================================

static constexpr float HIMM_MIN = 0.0f;
static constexpr float HIMM_MAX = 15.0f;

bool isFrontier(const CellKey& key)
{
    auto it = grid.find(key);

    if (it == grid.end())
        return false;

    const Cell& cell = it->second;

    if (!cell.properties.isFree)
        return false;

    for (int dx=-1; dx<=1; dx++)
    {
        for (int dy=-1; dy<=1; dy++)
        {
            if (dx == 0 && dy == 0)
                continue;

            CellKey n{
                key.x + dx,
                key.y + dy
            };

            if (grid.find(n) == grid.end())
            {
                return true;
            }
        }
    }

    return false;
}

void updateFrontierNeighborhood(
    const CellKey& center)
{
    for (int dx = -1; dx <= 1; ++dx)
    {
        for (int dy = -1; dy <= 1; ++dy)
        {
            CellKey key{
                center.x + dx,
                center.y + dy
            };

            auto it = grid.find(key);

            if (it == grid.end())
                continue;

            bool frontier =
                isFrontier(key);

            it->second.properties.isFrontier =
                frontier;

            if (frontier)
            {
                frontierSet.insert(key);
            }
            else
            {
                frontierSet.erase(key);
            }
        }
    }
}

void increaseOccupancy(const Cell& c)
{
    CellKey key{c.x, c.y};

    auto& stored = grid[key];

    stored.x = c.x;
    stored.y = c.y;

    stored.himm = std::min(
        HIMM_MAX,
        stored.himm + 3.0f
    );

    stored.properties.isOccupied =
        stored.himm >= 10.0f;

    stored.properties.isFree =
        stored.himm <= 5.0f;

    updateFrontierNeighborhood(key);
}

void increaseFree(const Cell& c)
{
    CellKey key{c.x, c.y};

    auto& stored = grid[key];

    stored.x = c.x;
    stored.y = c.y;

    stored.himm = std::max(
        HIMM_MIN,
        stored.himm - 1.0f
    );

    stored.properties.isOccupied =
        stored.himm >= 10.0f;

    stored.properties.isFree =
        stored.himm <= 5.0f;

    updateFrontierNeighborhood(key);
}
     

// ======================================================
// BRESENHAM SEM ALOCAÇÃO
// ======================================================

template<typename Callback>
void bresenham(
    const Cell& start,
    const Cell& end,
    Callback callback)
{
    int x0 = start.x;
    int y0 = start.y;

    int x1 = end.x;
    int y1 = end.y;

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);

    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;

    int err = dx - dy;

    while (true)
    {
        bool last =
            (x0 == x1 && y0 == y1);

        callback(x0, y0, last);

        if (last)
            break;

        int e2 = 2 * err;

        if (e2 > -dy)
        {
            err -= dy;
            x0 += sx;
        }

        if (e2 < dx)
        {
            err += dx;
            y0 += sy;
        }
    }
}

// ======================================================
// LASER
// ======================================================

void updateCellsFromLaser(
    float maxRange = 2.0f)
{
    const float fov = M_PI;

    const float angleStep =
        fov / static_cast<float>(lasers.size());

    const float startAngle =
        -fov / 2.0f;

    Cell robotCell =
        findCell(
            robotPosition.x,
            robotPosition.y);

    std::lock_guard<std::mutex> lock(
        visitedCellsMutex);

    for (size_t i = 0; i < lasers.size(); ++i)
    {
        float range = lasers[i];

        bool hitObstacle = true;

        if (range > maxRange)
        {
            range = maxRange;
            hitObstacle = false;
        }

        float angle =
            robotPosition.theta +
            startAngle +
            (lasers.size() - 1 - i) *
            angleStep;

        float xEnd =
            robotPosition.x +
            std::cos(angle) * range;

        float yEnd =
            robotPosition.y +
            std::sin(angle) * range;

        Cell endCell =
            findCell(xEnd, yEnd);

        bresenham(
            robotCell,
            endCell,
            [&](int x,
                int y,
                bool last)
        {
            Cell c;
            c.x = x;
            c.y = y;

            if (last)
            {
                if (hitObstacle)
                    increaseOccupancy(c);
            }
            else
            {
                increaseFree(c);
            }
        });
    }
}

// ======================================================
// HISTÓRICO
// ======================================================

void saveHistoryToFile(
    const std::string& filename)
{
    std::ofstream file(filename);

    if (!file.is_open())
        return;

    for (const auto& h : history)
    {
        file
            << h.pose.x << " "
            << h.pose.y << " "
            << h.pose.theta;

        for (float r : h.laserReadings)
        {
            file << " " << r;
        }

        file << "\n";
    }
}

// ======================================================
// IDENTIFICAR CELULAS VISITADAS
// ======================================================

std::vector<Cell> getVisitedCells()
{
    std::lock_guard<std::mutex> lock(visitedCellsMutex);

    std::vector<Cell> result;
    result.reserve(grid.size());

    for (const auto& [key, cell] : grid)
    {
        result.push_back(cell);
    }

    return result;
}

// ======================================================
// DETECÇÃO DE FRONTEIRAS
// ======================================================
std::vector<Cell> getFrontiers()
{
    std::lock_guard<std::mutex> lock(
        visitedCellsMutex);

    std::vector<Cell> result;

    result.reserve(frontierSet.size());

    for (const auto& key : frontierSet)
    {
        auto it = grid.find(key);

        if (it != grid.end())
        {
            result.push_back(
                it->second);
        }
    }

    return result;
}


// ======================================================
// THREAD
// ======================================================

void* mappingThreadFunction(void* arg)
{
    while (rclcpp::ok())
    {
        {
            std::lock_guard<std::mutex> lock(
                visitedCellsMutex);

            Cell robotCell =
                findCell(
                    robotPosition.x,
                    robotPosition.y);

            increaseFree(robotCell);
        }

        if (history.empty() ||
            !samePose(
                history.back().pose,
                robotPosition))
        {
            history.push_back(
            {
                robotPosition,
                lasers
            });

            if (history.size() > 10000)
            {
                history.erase(
                    history.begin());
            }
        }

        updateCellsFromLaser();

        usleep(20000); // 50Hz
    }

    return nullptr;
}