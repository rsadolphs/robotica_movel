#include "Pathing.hpp"
#include "Mapping.hpp"
#include "rclcpp/rclcpp.hpp"
#include "Globals.hpp"

#include <GLFW/glfw3.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <sstream>
#include <queue>
#include <climits>


// Variável global ou extern para compartilhar posição do robô
extern Position roboPosicao;
extern std::vector<float> laseres;
extern std::vector<std::vector<bool>> knownRegion;

extern std::vector<float> offset;
extern float scaleFactor;
// Cria Grade e Matrizes
extern GridInfo grid;
extern int size;  
extern std::vector<std::vector<float>> matrizMundo;
extern std::vector<std::vector<Cell>> occGrid;


// Estrutura para lista de pontos
std::vector<Ponto> listaPontos;
std::vector<Centroide> listaCentroides;
std::vector<std::pair<int,int>> caminhoRobo;
std::vector<std::pair<int,int>> caminhoClusterAB;
std::vector<std::pair<int,int>> caminhoCompleto;

double yawAstar = 0.0;


bool classificar(float valor) {
    if (valor <= 10.0f)
        return true;     // livre
    else
        return false;     // parede
}

void detectarFronteiras(std::vector<Ponto>& pontos)
{
    // Criar tabela rápida apenas para verificar existência de pontos conhecidos
    std::unordered_set<std::pair<int,int>, PairHash> tabela;
    tabela.reserve(pontos.size() * 2);
    for (const auto& p : pontos) tabela.insert({p.x, p.y});

    // 8 vizinhos (8-conectividade)
    const int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int dy[8] = {-1,-1,-1,  0, 0,  1, 1, 1};

    for (auto& p : pontos) {
        if (!p.isFree) { p.isFrontier = false; continue; }

        bool frontier = false;
        for (int k = 0; k < 8; ++k) {
            int nx = p.x + dx[k];
            int ny = p.y + dy[k];

            // Fora dos limites -> considere fronteira
            if (ny < 0 || nx < 0 ||
                ny >= static_cast<int>(knownRegion.size()) ||
                nx >= static_cast<int>(knownRegion[0].size()))
            {
                frontier = true;
                break;
            }

            // Se o vizinho ainda não foi visto (knownRegion == false) -> fronteira
            if (!knownRegion[ny][nx]) {
                frontier = true;
                break;
            }
        }
        p.isFrontier = frontier;
    }
}

std::vector<Ponto> gerarPontos(const std::vector<std::vector<float>>& matrizMundo) {
    std::vector<Ponto> pontos;
    if (matrizMundo.empty() || knownRegion.empty()) return pontos;

    size_t linhas = matrizMundo.size();
    size_t colunas = matrizMundo[0].size();
    pontos.reserve(linhas * colunas);

    for (size_t y = 0; y < linhas; ++y) {
        for (size_t x = 0; x < colunas; ++x) {
            // Só adiciona se for conhecido (visível)
            if (y >= knownRegion.size() || x >= knownRegion[y].size()) continue;
            if (!knownRegion[y][x]) continue; // pular desconhecidos

            float valorOriginal = matrizMundo[y][x];

            Ponto p;
            p.x = static_cast<int>(x);
            p.y = static_cast<int>(y);
            p.cluster = 0;
            p.isFree = classificar(valorOriginal);  // mapeia para livre/parede
            p.isFrontier = false;

            pontos.push_back(p);
        }
    }

    return pontos;
}

static float distancia(const Ponto& a, const Ponto& b) {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    return std::sqrt(dx*dx + dy*dy);
}

static std::vector<int> regionQuery(const std::vector<Ponto>& pontos, int idx, float eps) {
    std::vector<int> vizinhos;
    for (int i = 0; i < (int)pontos.size(); i++) {
        if (distancia(pontos[idx], pontos[i]) <= eps)
            vizinhos.push_back(i);
    }
    return vizinhos;
}

void expandCluster(
    std::vector<Ponto>& pontos,
    int idx,
    int clusterId,
    float eps,
    int minPts
) {
    std::vector<int> seeds = regionQuery(pontos, idx, eps);

    pontos[idx].cluster = clusterId;

    size_t i = 0;
    while (i < seeds.size()) {
        int p = seeds[i];

        if (pontos[p].cluster == -1)   // noise vira membro
            pontos[p].cluster = clusterId;

        if (pontos[p].cluster == 0) {  // não visitado
            pontos[p].cluster = clusterId;

            std::vector<int> result = regionQuery(pontos, p, eps);

            if (result.size() >= (size_t)minPts) {
                seeds.insert(seeds.end(), result.begin(), result.end());
            }
        }
        i++;
    }
}

void rodarDBSCAN(std::vector<Ponto>& pontos, float eps, int minPts) {
    int clusterId = 1;

    for (auto& p : pontos)
        p.cluster = 0; // 0 = não visitado /  -1 = noise

    for (int i = 0; i < (int)pontos.size(); i++) {
        if (pontos[i].cluster != 0)
            continue;

        auto vizinhos = regionQuery(pontos, i, eps);

        if ((int)vizinhos.size() < minPts) {
            pontos[i].cluster = -1; // noise
        } else {
            expandCluster(pontos, i, clusterId, eps, minPts);
            clusterId++;
        }
    }
}

std::vector<Centroide> calcularCentroides(const std::vector<Ponto>& listaPontos) {
    std::map<int, std::vector<Ponto>> clusters;

    // Agrupa pontos por clusterId
    for (const auto& p : listaPontos) {
        if (p.cluster >= 0) {   // clusters válidos
            clusters[p.cluster].push_back(p);
        }
    }

    std::vector<Centroide> centroides;

    // Calcula centroide de cada cluster
    for (auto& kv : clusters) {
        int id = kv.first;
        auto& pts = kv.second;

        long somaX = 0, somaY = 0;
        for (auto& p : pts) {
            somaX += p.x;
            somaY += p.y;
        }

        Centroide c;
        c.clusterId  = id;
        c.numPontos  = pts.size();
        c.x          = somaX / pts.size();
        c.y          = somaY / pts.size();
        c.distVizinho = -1;   // será calculado depois

        centroides.push_back(c);
    }

    return centroides;
}

void calcularDistanciasVizinho(std::vector<Centroide>& centroides) {
    if (centroides.size() <= 1) {
        return;
    }

    for (auto& c : centroides) {
        float minDist = std::numeric_limits<float>::max();

        for (const auto& outros : centroides) {
            if (c.clusterId == outros.clusterId)
                continue;

            float dx = c.x - outros.x;
            float dy = c.y - outros.y;
            float dist = std::sqrt(dx*dx + dy*dy);

            if (dist < minDist) {
                minDist = dist;
            }
        }

        c.distVizinho = minDist;
    }
}

inline double custoCelulaComInflacao(
    const std::vector<std::vector<Cell>>& grid,
    int y, int x,
    double cost_free,
    double cost_unk,
    double cont_infl,
    double cost_occ,
    int inflationRadius = 1)
{
    // Verificar vizinhos dentro do raio
    for (int dy = -inflationRadius; dy <= inflationRadius; dy++) {
        for (int dx = -inflationRadius; dx <= inflationRadius; dx++) {
            if (dx == 0 && dy == 0) continue;

            int ny = y + dy;
            int nx = x + dx;

            if (ny < 0 || nx < 0 || ny >= grid.size() || nx >= grid[0].size())
                continue;

            if (grid[ny][nx].isOcc) {
                // Inflar custo da célula atual, pois tem obstáculo ao lado
                return cont_infl;
            }
        }
    }

    if (grid[y][x].isOcc){
        return cost_occ;
    }
    else if (grid[y][x].isUnknown){
        return cost_unk;
    }
    else{
        return cost_free;
    }
}

//-----------------------------------------
// Heurísticas e A*
//-----------------------------------------
inline double heurOctile(int x, int y, int gx, int gy) {
    double dx = std::abs(x - gx);
    double dy = std::abs(y - gy);
    return (dx + dy) + (std::sqrt(2.0) - 2.0) * std::min(dx, dy);
}

double aStarBase(
    const std::vector<std::vector<Cell>>& grid,
    int sx, int sy,
    int gx, int gy,
    std::vector<std::pair<int,int>>& outPath,
    bool globalMode = false
){
    const int H = grid.size();
    if (H == 0) return -1;
    const int W = grid[0].size();
    if (W == 0) return -1;

    double cost_free = 1.0;
    double cost_unk = 50.0;
    double cont_infl = 1e4; 
    double cost_occ = 1e6;

    if(globalMode){
        cost_free  = 50.0;
        cost_unk   = 1.0;
    }

    auto inside = [&](int y, int x){
        return x >= 0 && x < W && y >= 0 && y < H;
    };

    const double INF = 1e9;

    std::vector<std::vector<double>> g(H, std::vector<double>(W, INF));
    std::vector<std::vector<std::pair<int,int>>> parent(
        H, std::vector<std::pair<int,int>>(W, {-1, -1})
    );

    using Node = std::tuple<double,double,int,int>; 
    // (f, g, y, x)
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;

    g[sy][sx] = 0.0;
    double h0 = heurOctile(sx, sy, gx, gy);
    pq.push({h0, 0.0, sy, sx});

    const int dx[8] = {1,-1,0,0, 1,1,-1,-1};
    const int dy[8] = {0,0,1,-1, 1,-1,1,-1};

    while (!pq.empty()) {
        auto [f, gc, y, x] = pq.top();
        pq.pop();

        if (gc > g[y][x]) continue;

        if (x == gx && y == gy) {
            outPath.clear();

            int cy = gy, cx = gx;
            while (!(cy == sy && cx == sx)) {
                outPath.push_back({cy, cx});
                auto [py, px] = parent[cy][cx];
                cy = py; 
                cx = px;
            }
            outPath.push_back({sy, sx});
            std::reverse(outPath.begin(), outPath.end());

            return g[gy][gx];
        }

        for (int k = 0; k < 8; k++) {
            int nx = x + dx[k];
            int ny = y + dy[k];

            if (!inside(ny, nx)) continue;

            double stepCost = (k < 4 ? 1.0 : std::sqrt(2.0));

            double w = custoCelulaComInflacao(
                grid,
                ny, nx,
                cost_free,
                cost_unk, 
                cost_occ,
                cont_infl,
                2
            );

            double ng = gc + stepCost * w;

            if (ng < g[ny][nx]) {
                g[ny][nx] = ng;

                parent[ny][nx] = {y, x};

                double h = heurOctile(nx, ny, gx, gy);
                double nf = ng + h;

                pq.push({nf, ng, ny, nx});
            }
        }
    }

    return -1;
}

std::vector<double> distRoboParaCentroidesAstar(
    const std::vector<std::vector<Cell>>& grid,
    int rx, int ry,
    const std::vector<Centroide>& cs,
    bool globalMode = false
){
    std::vector<double> v;
    v.reserve(cs.size());

    for (auto& c : cs) {
        std::vector<std::pair<int,int>> tmp;
        double d = aStarBase(grid, rx, ry, c.x, c.y, tmp, globalMode);
        v.push_back(d);
    }
    return v;
}

double cost3Points(
    const std::vector<std::vector<Cell>>& grid,
    int rx, int ry,
    const std::vector<Centroide>& cs,
    int i, int j
) {
    std::vector<std::pair<int,int>> p1, p2;

    double d1 = aStarBase(grid, rx, ry, cs[i].x, cs[i].y, p1, false);
    if (d1 <= 0) return 1e18;

    double d2 = aStarBase(grid, cs[i].x, cs[i].y, cs[j].x, cs[j].y, p2, true);
    if (d2 <= 0) return 1e18;

    return d1 + d2;
}

std::pair<int,int> escolherABeBAstar(
    const std::vector<std::vector<Cell>>& grid,
    int rx, int ry,
    const std::vector<Centroide>& cs)
{
    if (cs.size() < 2) return {-1,-1};

    // Histerese
    static int lastA = -1;
    static int lastB = -1;
    static double lastCost = 1e18;

    double bestCost = 1e18;
    int bestI = -1;
    int bestJ = -1;

    // Busca par ótimo
    for (int i = 0; i < (int)cs.size(); i++) {
        for (int j = 0; j < (int)cs.size(); j++) {
            if (i == j) continue;

            double cost = cost3Points(grid, rx, ry, cs, i, j);
            if (cost < bestCost) {
                bestCost = cost;
                bestI = i;
                bestJ = j;
            }
        }
    }

    // Se nunca escolheu um par, aceita o primeiro sem histerese
    if (lastA == -1 || lastB == -1) {
        lastA = bestI;
        lastB = bestJ;
        lastCost = bestCost;
        return {bestI, bestJ};
    }

    // Histerese: só troca se a melhoria for significativa
    const double threshold = 0.90;  // 10% melhor
    if (bestCost < lastCost * threshold) {
        // Melhorou bastante -> aceitar troca
        lastA = bestI;
        lastB = bestJ;
        lastCost = bestCost;
        return {bestI, bestJ};
    }

    // Caso contrário, mantemos o par anterior
    return {lastA, lastB};
}

bool gerarCaminhoAstarCompleto(
    const std::vector<std::vector<Cell>>& grid,
    int rx, int ry,
    const std::vector<Centroide>& cs,
    std::vector<std::pair<int,int>>& outFinal)
{
    outFinal.clear();

    auto [iA, iB] = escolherABeBAstar(grid, rx, ry, cs);
    if (iA < 0 || iB < 0) return false;

    int Ax = cs[iA].x;
    int Ay = cs[iA].y;
    int Bx = cs[iB].x;
    int By = cs[iB].y;

    std::vector<std::pair<int,int>> pathR;
    std::vector<std::pair<int,int>> pathAB;

    if (aStarBase(grid, rx, ry, Ax, Ay, pathR) < 0)
        return false;

    if (aStarBase(grid, Ax, Ay, Bx, By, pathAB) < 0)
        return false;

    outFinal = pathR;
    outFinal.insert(outFinal.end(), pathAB.begin() + 1, pathAB.end());

    return true;
}

double calcularYawParaCaminho(
    const std::vector<std::pair<int,int>>& caminhoCompleto,
    const MatrixPosition& matPosRobo)
{
    if (caminhoCompleto.empty()) return 0.0;

    // --- 1. Coletar até 5 pontos do caminho ---
    int N = std::min(5, (int)caminhoCompleto.size());

    // Usar média para regressão
    double sumX = 0, sumY = 0;
    for (int i = 0; i < N; i++) {
        sumX += caminhoCompleto[i].second; // coluna -> x
        sumY += caminhoCompleto[i].first;  // linha  -> y
    }

    double meanX = sumX / N;
    double meanY = sumY / N;

    // --- 2. Calcular coeficiente da regressão linear ---
    double num = 0, den = 0;
    for (int i = 0; i < N; i++) {
        double x = caminhoCompleto[i].second;
        double y = caminhoCompleto[i].first;

        num += (x - meanX) * (y - meanY);
        den += (x - meanX) * (x - meanX);
    }

    if (den == 0) {
        // Caso degenerado (todos x iguais)
        // direção vertical
        double dy = meanY - matPosRobo.linha;
        return (dy >= 0) ? (M_PI/2) : (-M_PI/2);
    }

    double slope = num / den;  // inclinação da reta

    // --- 3. Vetor direção da reta ---

    // Tomamos um ponto um pouco à frente na reta
    double x2 = meanX + 1.0;         
    double y2 = meanY + slope * (x2 - meanX);

    // Vetor direção relativo ao robô
    double dx = x2 - matPosRobo.coluna;
    double dy = y2 - matPosRobo.linha;

    // --- 4. Calcular yaw desejado ---
    double yaw = std::atan2(dy, dx); // radianos

    return yaw;
}


void* pathingThreadFunction(void* arg) {

    while (rclcpp::ok()) {

        if (!matrizMundo.empty() && !occGrid.empty()){

            MatrixPosition matPosRobo = findCell(
                                            roboPosicao.x * scaleFactor - offset[0], 
                                            roboPosicao.y * scaleFactor - offset[1],        
                                            grid.inicio, 
                                            grid.passo
                                        );
            
            int linhas = matrizMundo.size();
            int colunas = matrizMundo[0].size();
            
            if (posicaoValida(matPosRobo, linhas, colunas)){
                        
                listaPontos = gerarPontos(matrizMundo);

                if (!listaPontos.empty()){

                    detectarFronteiras(listaPontos);
                    std::vector<Ponto> fronteiras;
                    for (auto& p : listaPontos)
                        if (p.isFrontier)
                            fronteiras.push_back(p);

                    rodarDBSCAN(fronteiras, 5.0f, 10); 

                    listaCentroides = calcularCentroides(fronteiras);

                    calcularDistanciasVizinho(listaCentroides);

                    std::cout << "Centroides encontrados: " << listaCentroides.size() << "\n";

                    for (auto& c : listaCentroides) {
                        std::cout << "Cluster: " << c.clusterId << " (" << c.x << ", " << c.y << ") " << " Size: " << c.numPontos << " NN: " << c.distVizinho << "\n";
                    }
                    /*
                    std::vector<double> distancias = calcularDistanciasRoboParaCentroides(occGrid, matPosRobo.coluna, matPosRobo.linha, listaCentroides);
                    for (size_t i = 0; i < distancias.size(); i++) {
                        std::cout << "Cluster " << listaCentroides[i].clusterId
                                << " | Distancia = " << distancias[i] << std::endl;
                    }   
                    */
                    gerarCaminhoAstarCompleto(occGrid, matPosRobo.coluna, matPosRobo.linha, listaCentroides, caminhoCompleto);

                    yawAstar = calcularYawParaCaminho(caminhoCompleto, matPosRobo);
                }
            }
        }
        // Pequena pausa para não sobrecarregar a CPU
        usleep(1000000); // 1000ms
    }

    return NULL;
}