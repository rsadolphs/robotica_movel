#ifndef EXPLORER_HPP
#define EXPLORER_HPP

#include "Mapping.hpp"

#include <vector>

struct FrontierCluster
{
    std::vector<Cell> cells;

    Cell frontierCenter;

    Cell freeCentroid;

    float centroidX = 0.0f;
    float centroidY = 0.0f;
};

std::vector<FrontierCluster>
detectFrontierClusters();

#endif