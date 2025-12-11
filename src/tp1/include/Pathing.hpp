// pathing.hpp
#ifndef PATHING_HPP
#define PATHING_HPP

#include <vector>


void* pathingThreadFunction(void* arg);


struct Ponto {
    int x;                   // coluna
    int y;                   // linha
    int cluster;
    bool isFree     = false; 
    bool isFrontier = false;
};

struct PairHash {
    size_t operator()(const std::pair<int,int>& p) const {
        // slim 64-bit mix
        return (static_cast<size_t>(p.first) << 32) ^ static_cast<size_t>(p.second);
    }
};

struct Centroide {
    int clusterId;          // Número do cluster (mesmo ID usado em listaPontos)
    int x;                  // Coordenada X do centroide
    int y;                  // Coordenada Y do centroide
    int numPontos;          // Quantidade de pontos no cluster
    float distVizinho;      // Distância até o centroide mais próximo
};

struct Cell {
    bool isUnknown = false;
    bool isFree    = false;
    bool isOcc     = false;
};

struct PathResult {
    double custo;
    std::vector<std::pair<int,int>> caminho; // (y,x) do início ao fim
};

struct CaminhoInfo {
    double custo;
    std::vector<std::pair<int,int>> path;
    bool valido = false;
};


#endif // PATHING_HPP
