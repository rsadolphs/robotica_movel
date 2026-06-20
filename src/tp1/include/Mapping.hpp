#ifndef MAPPING_HPP
#define MAPPING_HPP

#include <vector>
#include <string>

void* mappingThreadFunction(void* arg);

struct Position{
    float x, y, theta;
};

struct CellProperties{
    bool isFree     = false;
    bool isOccupied = false;
    bool isUnknown  = false;
    bool isFrontier = false;
};

struct Cell {
    int x;
    int y;
    float himm = 7.5f; 
    CellProperties properties;
};

struct History{
    Position pose;
    std::vector<float> laserReadings;
};


std::vector<Cell> getVisitedCells();
std::vector<Cell> getFrontiers();

void saveHistoryToFile(const std::string& filename);

// tamanho da célula em centímetros (definido em Mapping.cpp)
extern int cellSizeCentimeters;

#endif // MAPPING_HPP