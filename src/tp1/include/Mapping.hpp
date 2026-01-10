#ifndef MAPPING_HPP
#define MAPPING_HPP

#include <vector>

void* mappingThreadFunction(void* arg);

struct Position{
    float x, y, theta;
};

struct CellProperties{
    bool isFree     = false;
    bool isOccupied = false;
    bool isUnknown  = false;
};

struct Cell {
    int x;
    int y;
    float himm = 7.5f; 
    CellProperties properties;
};



#endif // MAPPING_HPP