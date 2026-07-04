#include "Potential.hpp"
#include "Mapping.hpp"
#include <cmath>
#include <mutex>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>

namespace Potential {

static std::mutex poseMutex;
static std::mutex stateMutex;

static float robotX = 0.0f;
static float robotY = 0.0f;
static float robotTheta = 0.0f;

static TargetYaw latestTarget;
static FieldState latestField;

static void getFieldBounds(
    const std::vector<Cell>& visitedCells,
    int& minX, int& minY,
    int& maxX, int& maxY)
{
    if (visitedCells.empty())
    {
        minX = minY = -10;
        maxX = maxY = 10;
        return;
    }

    minX = visitedCells[0].x;
    maxX = visitedCells[0].x;
    minY = visitedCells[0].y;
    maxY = visitedCells[0].y;

    for (const auto& cell : visitedCells)
    {
        minX = std::min(minX, cell.x);
        maxX = std::max(maxX, cell.x);
        minY = std::min(minY, cell.y);
        maxY = std::max(maxY, cell.y);
    }
}

static inline int fieldIndex(int x, int y, int minX, int minY, int width)
{
    return (y - minY) * width + (x - minX);
}

static FieldState computeFieldState(
    const std::vector<Cell>& visitedCells,
    const std::vector<Cell>& frontiers)
{
    FieldState state;
    if (visitedCells.empty())
        return state;

    int minX, minY, maxX, maxY;
    getFieldBounds(visitedCells, minX, minY, maxX, maxY);

    state.minX = minX;
    state.minY = minY;
    state.maxX = maxX;
    state.maxY = maxY;
    state.width = maxX - minX + 1;
    state.height = maxY - minY + 1;
    state.values.assign(state.width * state.height, 0.5f);
    state.active.assign(state.width * state.height, 0);

    std::vector<char> fixed(state.width * state.height, 0);

    for (const auto& cell : visitedCells)
    {
        int idx = fieldIndex(cell.x, cell.y, minX, minY, state.width);
        if (cell.properties.isOccupied)
        {
            state.values[idx] = 1.0f;
            fixed[idx] = 1;
            state.active[idx] = 1;
        }
        else if (cell.properties.isFrontier)
        {
            state.values[idx] = 0.0f;
            fixed[idx] = 1;
            state.active[idx] = 1;
        }
        else if (cell.properties.isFree)
        {
            state.values[idx] = 0.5f;
            fixed[idx] = 0;
            state.active[idx] = 1;
        }
        else
        {
            // unknown / unobserved cell: leave inactive
            state.values[idx] = 0.5f;
            fixed[idx] = 0;
            state.active[idx] = 0;
        }
    }

    for (const auto& frontier : frontiers)
    {
        int idx = fieldIndex(frontier.x, frontier.y, minX, minY, state.width);
        state.values[idx] = 0.0f;
        fixed[idx] = 1;
        state.active[idx] = 1;
    }

    std::vector<float> nextValues = state.values;
    const int maxIterations = 200;
    const float tolerance = 1e-4f;

    for (int iter = 0; iter < maxIterations; ++iter)
    {
        float maxChange = 0.0f;

        for (int y = minY; y <= maxY; ++y)
        {
            for (int x = minX; x <= maxX; ++x)
            {
                int idx = fieldIndex(x, y, minX, minY, state.width);
            if (!state.active[idx] || fixed[idx])
                continue;

            float sum = 0.0f;
            int count = 0;

            if (x > minX)
            {
                int nidx = idx - 1;
                if (state.active[nidx]) { sum += state.values[nidx]; count += 1; }
            }
            if (x < maxX)
            {
                int nidx = idx + 1;
                if (state.active[nidx]) { sum += state.values[nidx]; count += 1; }
            }
            if (y > minY)
            {
                int nidx = idx - state.width;
                if (state.active[nidx]) { sum += state.values[nidx]; count += 1; }
            }
            if (y < maxY)
            {
                int nidx = idx + state.width;
                if (state.active[nidx]) { sum += state.values[nidx]; count += 1; }
            }

            if (count > 0)
            {
                float value = sum / static_cast<float>(count);
                nextValues[idx] = value;
                maxChange = std::max(maxChange, std::fabs(value - state.values[idx]));
            }
        }
        }

        state.values.swap(nextValues);

        if (maxChange < tolerance)
            break;
    }

    state.valid = true;
    return state;
}

static float sampleField(const FieldState& state, float x, float y)
{
    if (!state.valid)
        return 0.5f;

    if (x < state.minX) x = state.minX;
    if (x > state.maxX) x = state.maxX;
    if (y < state.minY) y = state.minY;
    if (y > state.maxY) y = state.maxY;

    float fx = std::floor(x);
    float fy = std::floor(y);
    float cx = fx + 1.0f;
    float cy = fy + 1.0f;

    float wx = x - fx;
    float wy = y - fy;

    int x0 = static_cast<int>(fx);
    int y0 = static_cast<int>(fy);
    int x1 = static_cast<int>(cx);
    int y1 = static_cast<int>(cy);

    if (x1 > state.maxX) x1 = state.maxX;
    if (y1 > state.maxY) y1 = state.maxY;

    auto sampleCorner = [&](int xi, int yi) {
        int idx = fieldIndex(xi, yi, state.minX, state.minY, state.width);
        return state.active[idx] ? state.values[idx] : 0.0f;
    };

    auto activeCorner = [&](int xi, int yi) {
        int idx = fieldIndex(xi, yi, state.minX, state.minY, state.width);
        return state.active[idx];
    };

    float v00 = sampleCorner(x0, y0);
    float v10 = sampleCorner(x1, y0);
    float v01 = sampleCorner(x0, y1);
    float v11 = sampleCorner(x1, y1);

    int count = 0;
    count += activeCorner(x0, y0);
    count += activeCorner(x1, y0);
    count += activeCorner(x0, y1);
    count += activeCorner(x1, y1);

    if (count == 0)
        return 0.5f;

    float v0 = ((activeCorner(x0, y0) ? v00 * (1.0f - wx) : 0.0f) +
                (activeCorner(x1, y0) ? v10 * wx : 0.0f));
    float w0 = ((activeCorner(x0, y0) ? (1.0f - wx) : 0.0f) +
                (activeCorner(x1, y0) ? wx : 0.0f));

    float v1 = ((activeCorner(x0, y1) ? v01 * (1.0f - wx) : 0.0f) +
                (activeCorner(x1, y1) ? v11 * wx : 0.0f));
    float w1 = ((activeCorner(x0, y1) ? (1.0f - wx) : 0.0f) +
                (activeCorner(x1, y1) ? wx : 0.0f));

    float result0 = w0 > 0.0f ? v0 / w0 : 0.5f;
    float result1 = w1 > 0.0f ? v1 / w1 : 0.5f;
    float totalW = (w0 + w1) * 0.5f;
    if (totalW <= 0.0f)
        return 0.5f;

    return result0 * (1.0f - wy) + result1 * wy;
}

static TargetYaw computeTargetYaw(const FieldState& state)
{
    TargetYaw target;
    if (!state.valid)
        return target;

    std::lock_guard<std::mutex> lock(poseMutex);

    float rx = robotX;
    float ry = robotY;

    if (rx < state.minX || rx > state.maxX || ry < state.minY || ry > state.maxY)
        return target;

    float gradX = 0.0f;
    float gradY = 0.0f;

    // 1-cell radius central difference gradient
    gradX = (sampleField(state, rx + 1.0f, ry) - sampleField(state, rx - 1.0f, ry)) * 0.5f;
    gradY = (sampleField(state, rx, ry + 1.0f) - sampleField(state, rx, ry - 1.0f)) * 0.5f;

    float targetX = -gradX;
    float targetY = -gradY;
    float norm = std::hypot(targetX, targetY);

    if (norm < 1e-4f)
        return target;

    targetX /= norm;
    targetY /= norm;
    target.yaw = std::atan2(targetY, targetX);
    target.valid = true;
    return target;
}

void updateRobotPose(float x, float y, float theta)
{
    std::lock_guard<std::mutex> lock(poseMutex);
    robotX = x * 100.0f / cellSizeCentimeters;
    robotY = y * 100.0f / cellSizeCentimeters;
    robotTheta = theta;
}

TargetYaw getLatestTargetYaw()
{
    std::lock_guard<std::mutex> lock(stateMutex);
    return latestTarget;
}

FieldState getLatestFieldState()
{
    std::lock_guard<std::mutex> lock(stateMutex);
    return latestField;
}

void* potentialThreadFunction(void* arg)
{
    (void)arg;

    while (rclcpp::ok())
    {
        std::vector<Cell> visitedCells = getVisitedCells();
        std::vector<Cell> frontiers = getFrontiers();

        FieldState newField = computeFieldState(visitedCells, frontiers);
        TargetYaw newTarget = computeTargetYaw(newField);

        {
            std::lock_guard<std::mutex> lock(stateMutex);
            latestField = std::move(newField);
            latestTarget = newTarget;
        }

        usleep(40000); // 25 Hz
    }

    return nullptr;
}

} // namespace Potential
