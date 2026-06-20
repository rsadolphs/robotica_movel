#include "Explorer.hpp"
#include "rclcpp/rclcpp.hpp"

#include <queue>
#include <unordered_set>
#include <cmath>
#include <unistd.h>

struct FrontierKey
{
    int x;
    int y;

    bool operator==(const FrontierKey& other) const
    {
        return x == other.x &&
               y == other.y;
    }
};

struct FrontierKeyHash
{
    std::size_t operator()(
        const FrontierKey& k) const
    {
        return std::hash<int>()(k.x) ^
              (std::hash<int>()(k.y) << 1);
    }
};

static Cell findNearestFreeCell(
    float cx,
    float cy,
    const std::vector<Cell>& mapCells)
{
    float bestDist = 1e9f;

    Cell best;

    bool found = false;

    for (const auto& cell : mapCells)
    {
        if (!cell.properties.isFree)
            continue;

        float dx = cell.x - cx;
        float dy = cell.y - cy;

        float dist =
            dx * dx +
            dy * dy;

        if (dist < bestDist)
        {
            bestDist = dist;
            best = cell;
            found = true;
        }
    }

    if (!found)
    {
        best.x = std::round(cx);
        best.y = std::round(cy);
    }

    return best;
}

std::vector<FrontierCluster>
detectFrontierClusters()
{
    std::vector<FrontierCluster> clusters;

    std::vector<Cell> frontiers =
        getFrontiers();

    std::vector<Cell> mapCells =
        getVisitedCells();

    if (frontiers.empty())
        return clusters;

    std::unordered_set<
        FrontierKey,
        FrontierKeyHash> frontierLookup;

    for (const auto& f : frontiers)
    {
        frontierLookup.insert(
        {
            f.x,
            f.y
        });
    }

    std::unordered_set<
        FrontierKey,
        FrontierKeyHash> visited;

    for (const auto& start : frontiers)
    {
        FrontierKey startKey
        {
            start.x,
            start.y
        };

        if (visited.count(startKey))
            continue;

        FrontierCluster cluster;

        std::queue<FrontierKey> q;

        q.push(startKey);

        visited.insert(startKey);

        while (!q.empty())
        {
            FrontierKey current =
                q.front();

            q.pop();

            for (const auto& f : frontiers)
            {
                if (f.x == current.x &&
                    f.y == current.y)
                {
                    cluster.cells.push_back(f);
                    break;
                }
            }

            for (int dx = -1; dx <= 1; ++dx)
            {
                for (int dy = -1; dy <= 1; ++dy)
                {
                    if (dx == 0 &&
                        dy == 0)
                        continue;

                    FrontierKey neighbor
                    {
                        current.x + dx,
                        current.y + dy
                    };

                    if (
                        frontierLookup.count(neighbor) &&
                        !visited.count(neighbor)
                    )
                    {
                        visited.insert(
                            neighbor);

                        q.push(
                            neighbor);
                    }
                }
            }
        }

        float sumX = 0.0f;
        float sumY = 0.0f;

        for (const auto& cell :
             cluster.cells)
        {
            sumX += cell.x;
            sumY += cell.y;
        }

        cluster.centroidX =
            sumX /
            cluster.cells.size();

        cluster.centroidY =
            sumY /
            cluster.cells.size();

        float bestDist = 1e9f;

        for (const auto& cell :
             cluster.cells)
        {
            float dx =
                cell.x -
                cluster.centroidX;

            float dy =
                cell.y -
                cluster.centroidY;

            float dist =
                dx * dx +
                dy * dy;

            if (dist < bestDist)
            {
                bestDist = dist;

                cluster.frontierCenter =
                    cell;
            }
        }

        cluster.freeCentroid =
            findNearestFreeCell(
                cluster.centroidX,
                cluster.centroidY,
                mapCells);

        clusters.push_back(
            cluster);
    }

    return clusters;
}

void logClusterInfo(std::vector<FrontierCluster> clusters){
    std::cout
    << "\nClusters encontrados: "
    << clusters.size()
    << "\n";

    for(size_t i = 0; i < clusters.size(); ++i)
    {
        const auto& cluster =
            clusters[i];

        std::cout
            << "Cluster "
            << i
            << "\n";

        std::cout
            << "  Tamanho: "
            << cluster.cells.size()
            << "\n";

        std::cout
            << "  Centroide geometrico: ("
            << cluster.centroidX
            << ", "
            << cluster.centroidY
            << ")\n";

        std::cout
            << "  Centroide livre: ("
            << cluster.freeCentroid.x
            << ", "
            << cluster.freeCentroid.y
            << ")\n";

        std::cout
            << "  Fronteira central: ("
            << cluster.frontierCenter.x
            << ", "
            << cluster.frontierCenter.y
            << ")\n";
    }
}

void* explorerThreadFunction(void* arg)
{
    while(rclcpp::ok())
    {
        auto clusters =
            detectFrontierClusters();

        logClusterInfo(clusters);

        // escolher melhor cluster

        usleep(500000); // 2 Hz
    }

    return nullptr;
}