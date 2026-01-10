#ifndef MAPPING_HPP
#define MAPPING_HPP

#include <vector>

void* mappingThreadFunction(void* arg);

struct Cell {
    int x;
    int y;
    CellProperties properties;
};

struct CellProperties{
    bool isFree     = false;
    bool isOccupied = false;
    bool isUnknown  = false;
};

#endif // MAPPING_HPP