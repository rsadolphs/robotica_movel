#include "Potential.hpp"
#include "Mapping.hpp"
#include <algorithm>
#include <cmath>
#include <mutex>
#include <queue>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>

namespace Potential {

static std::mutex poseMutex;
static std::mutex stateMutex;

static float robotX = 0.0f;
static float robotY = 0.0f;
static float robotTheta = 0.0f;
static float robotVisionRadius = 20.0f; // cells
static float leftZoneFactor = 1.0f;
static float rightZoneFactor = 1.0f;
static float currentDirectionalBias = 0.0f;

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

static float normalizeAngle(float angle)
{
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle <= -M_PI) angle += 2.0f * M_PI;
    return angle;
}

static FieldState computeFieldState(
    const std::vector<Cell>& visitedCells,
    const std::vector<Cell>& frontiers)
{
    // Solve discrete Laplace equation with Dirichlet BC (occupied=1, frontier=0)
    // using Gauss-Seidel with SOR (over-relaxation). This follows the idea
    // used in classical potential field approaches (e.g. Prestes 2003).
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

    // Initialize field values and Dirichlet boundaries
    for (const auto& cell : visitedCells)
    {
        int idx = fieldIndex(cell.x, cell.y, minX, minY, state.width);
        if (cell.properties.isOccupied)
        {
            state.values[idx] = 1.0f; // high potential on obstacles
            state.active[idx] = 1;
            fixed[idx] = 1;
        }
        else if (cell.properties.isFrontier)
        {
            state.values[idx] = 0.0f; // low potential on frontiers (goals)
            state.active[idx] = 1;
            fixed[idx] = 1;
        }
        else if (cell.properties.isFree)
        {
            state.values[idx] = 0.5f; // initial guess for free cells
            state.active[idx] = 1;
            fixed[idx] = 0;
        }
        else
        {
            // unknown / unobserved cell: leave inactive
            state.values[idx] = 0.5f;
            state.active[idx] = 0;
            fixed[idx] = 0;
        }
    }

    // Enforce frontier cells as Dirichlet (in case frontiers include new cells)
    for (const auto& frontier : frontiers)
    {
        int idx = fieldIndex(frontier.x, frontier.y, minX, minY, state.width);
        if (!state.active[idx])
            continue;
        state.values[idx] = 0.0f;
        fixed[idx] = 1;
    }

    // SOR parameters: compute optimal omega using spectral radius
    // Formula (from Prestes 2003 / SOR theory):
    // rho = 0.5 * [cos(pi/Lx) + cos(pi/Ly)]
    // omega = 2 / (1 + sqrt(1 - rho^2))
    float Lx = static_cast<float>(state.width);
    float Ly = static_cast<float>(state.height);
    
    float rho = 0.5f * (std::cos(M_PI / Lx) + std::cos(M_PI / Ly));
    float rho_sq = rho * rho;
    
    // Clamp rho^2 to avoid sqrt of negative (edge cases in very small grids)
    rho_sq = std::max(0.0f, std::min(0.9999f, rho_sq));
    
    float omega = 2.0f / (1.0f + std::sqrt(1.0f - rho_sq));
    omega = std::min(1.99f, std::max(1.0f, omega)); // keep in (1, 2) range
    
    const int maxIterations = 80; // allow more iterations with optimal omega
    const float tolerance = 1e-5f;

    for (int iter = 0; iter < maxIterations; ++iter)
    {
        float maxChange = 0.0f;

        for (int y = state.minY; y <= state.maxY; ++y)
        {
            for (int x = state.minX; x <= state.maxX; ++x)
            {
                int idx = fieldIndex(x, y, minX, minY, state.width);
                if (!state.active[idx] || fixed[idx])
                    continue;

                float sum = 0.0f;
                int count = 0;

                // 4-neighbors
                if (x > state.minX)
                {
                    int nidx = idx - 1;
                    if (state.active[nidx]) { sum += state.values[nidx]; ++count; }
                }
                if (x < state.maxX)
                {
                    int nidx = idx + 1;
                    if (state.active[nidx]) { sum += state.values[nidx]; ++count; }
                }
                if (y > state.minY)
                {
                    int nidx = idx - state.width;
                    if (state.active[nidx]) { sum += state.values[nidx]; ++count; }
                }
                if (y < state.maxY)
                {
                    int nidx = idx + state.width;
                    if (state.active[nidx]) { sum += state.values[nidx]; ++count; }
                }

                if (count == 0)
                    continue;

                float newVal = sum / static_cast<float>(count);
                // SOR update
                float updated = state.values[idx] + omega * (newVal - state.values[idx]);
                maxChange = std::max(maxChange, std::fabs(updated - state.values[idx]));
                state.values[idx] = updated;
            }
        }

        if (maxChange < tolerance)
            break;
    }

    // Diagnostics: compute statistics to detect pathological cases
    int countActive = 0;
    int countFixed = 0;
    float minVal = 1e9f, maxVal = -1e9f, sumVal = 0.0f;
    for (size_t i = 0; i < state.values.size(); ++i)
    {
        if (state.active[i])
        {
            ++countActive;
            if (fixed[i]) ++countFixed;
            float v = state.values[i];
            minVal = std::min(minVal, v);
            maxVal = std::max(maxVal, v);
            sumVal += v;
        }
    }

    if (countActive > 0)
    {
        float mean = sumVal / static_cast<float>(countActive);
        if (countFixed == countActive || mean > 0.95f)
        {
            RCLCPP_WARN(rclcpp::get_logger("Potential"),
                        "Field all-fixed/near-1 (ω=%.3f fixed=%d active=%d mean=%.3f min=%.3f max=%.3f)",
                        omega, countFixed, countActive, mean, minVal, maxVal);
        }
        else if (mean > 0.8f)
        {
            RCLCPP_INFO(rclcpp::get_logger("Potential"),
                        "Field high (ω=%.3f mean=%.3f fixed=%d/%d)",
                        omega, mean, countFixed, countActive);
        }
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

    float sampled = result0 * (1.0f - wy) + result1 * wy;
    return sampled;
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

    float fx = std::cos(robotTheta);
    float fy = std::sin(robotTheta);
    float lx = -fy;
    float ly = fx;

    float sampleForward = sampleField(state, rx + fx, ry + fy);
    float sampleBackward = sampleField(state, rx - fx, ry - fy);
    float sampleLeft = sampleField(state, rx + lx, ry + ly);
    float sampleRight = sampleField(state, rx - lx, ry - ly);

    float gradForward = (sampleForward - sampleBackward) * 0.5f;
    float gradSide = (sampleLeft - sampleRight) * 0.5f;

    float targetForward = -gradForward;
    float targetLeft = -gradSide;

    float targetX = targetForward * fx + targetLeft * lx;
    float targetY = targetForward * fy + targetLeft * ly;
    float norm = std::hypot(targetX, targetY);

    if (norm < 1e-4f)
        return target;

    targetX /= norm;
    targetY /= norm;
    float rawYaw = std::atan2(targetY, targetX);

    // Apply a small directional bias as an angular offset rather than warping the entire field.
    // Positive bias means prefer the right side, so rotate the target yaw slightly clockwise.
    float maxBiasAngle = M_PI / 12.0f; // 15 degrees max
    float biasAngle = currentDirectionalBias * maxBiasAngle;
    rawYaw = normalizeAngle(rawYaw - biasAngle);

    target.yaw = rawYaw;
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

void setDirectionalPreference(float leftFactor, float rightFactor)
{
    std::lock_guard<std::mutex> lock(stateMutex);
    leftZoneFactor = leftFactor;
    rightZoneFactor = rightFactor;
}

void setDirectionalBias(float bias)
{
    std::lock_guard<std::mutex> lock(stateMutex);
    constexpr float scale = 0.35f;
    float clamped = std::max(-1.0f, std::min(1.0f, bias));
    currentDirectionalBias = clamped;
    // Positive bias should prefer the right side, so lower the potential on the right
    // and raise it on the left. The robot then moves toward lower potential values.
    leftZoneFactor = std::clamp(1.0f + clamped * scale, 0.5f, 2.0f);
    rightZoneFactor = std::clamp(1.0f - clamped * scale, 0.5f, 2.0f);
}

void setVisionRadius(float radiusCells)
{
    std::lock_guard<std::mutex> lock(stateMutex);
    robotVisionRadius = radiusCells;
}

float getVisionRadius()
{
    std::lock_guard<std::mutex> lock(stateMutex);
    return robotVisionRadius;
}

float getDirectionalBias()
{
    std::lock_guard<std::mutex> lock(stateMutex);
    return currentDirectionalBias;
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
