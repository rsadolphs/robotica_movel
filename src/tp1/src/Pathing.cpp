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

// Custos configuráveis
const double COST_FREE      = 20.0;
const double COST_UNKNOWN   = 1.0;
const double COST_INFLATED  = 1e9; 
const double COST_OCCUPIED  = 1e9;   


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
    double COST_FREE,
    double COST_UNKNOWN,
    double COST_OCCUPIED,
    double COST_INFLATED,
    int inflationRadius = 1)
{
    // Se a célula já é ocupada → custo máximo
    if (grid[y][x].isOcc)
        return COST_OCCUPIED;

    // Se a célula é unknown → custo intermediário
    if (grid[y][x].isUnknown)
        return COST_UNKNOWN;

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
                return COST_INFLATED;
            }
        }
    }

    // Caso normal → célula livre sem obstáculo perto
    return COST_FREE;
}



//-----------------------------------------
// DIJKSTRA BASE
//-----------------------------------------
double dijkstraBase(
    const std::vector<std::vector<Cell>>& grid,
    int sx, int sy,
    int gx, int gy,
    std::vector<std::pair<int,int>>& outPath)
{
    const int H = grid.size();
    if (H == 0) return -1;
    const int W = grid[0].size();
    if (W == 0) return -1;

    auto inside = [&](int y, int x){
        return x >= 0 && x < W && y >= 0 && y < H;
    };

    const double INF = 1e18;

    std::vector<std::vector<double>> dist(H, std::vector<double>(W, INF));
    std::vector<std::vector<std::pair<int,int>>> parent(
        H, std::vector<std::pair<int,int>>(W, {-1, -1})
    );

    using Node = std::tuple<double,int,int>;
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;

    dist[sy][sx] = 0.0;
    pq.push({0.0, sy, sx});

    const int dx[8] = {1,-1,0,0, 1,1,-1,-1};
    const int dy[8] = {0,0,1,-1, 1,-1,1,-1};

    while (!pq.empty()) {
        auto [d, y, x] = pq.top();
        pq.pop();

        if (d > dist[y][x]) continue;

        if (x == gx && y == gy) {
            outPath.clear();

            int cy = gy, cx = gx;
            while (!(cy == sy && cx == sx)) {
                outPath.push_back({cy, cx});
                auto [py, px] = parent[cy][cx];
                if (py == -1) break;
                cy = py; cx = px;
            }
            outPath.push_back({sy, sx});
            std::reverse(outPath.begin(), outPath.end());
            return d;
        }

        for (int k = 0; k < 8; k++) {
            int nx = x + dx[k];
            int ny = y + dy[k];

            if (!inside(ny, nx)) continue;

            double mc = (k < 4 ? 1.0 : sqrt(2.0));
            double w = custoCelulaComInflacao(
                        grid,
                        ny, nx,
                        COST_FREE,
                        COST_UNKNOWN,
                        COST_OCCUPIED,
                        COST_INFLATED,   // novo custo inflado
                        3                // raio da inflação (1 = vizinhos imediatos)
                    );

            double nd = d + mc * w;
            if (nd < dist[ny][nx]) {
                dist[ny][nx] = nd;
                parent[ny][nx] = {y, x};
                pq.push({nd, ny, nx});
            }
        }
    }

    return -1; // sem caminho
}

//-----------------------------------------
// DISTÂNCIA DO ROBO A OS CENTROIDES
//-----------------------------------------
std::vector<double> calcularDistanciasRoboParaCentroides(
    const std::vector<std::vector<Cell>>& grid,
    int rx, int ry,
    const std::vector<Centroide>& centroides)
{
    std::vector<double> v;
    v.reserve(centroides.size());

    for (auto& c : centroides) {
        std::vector<std::pair<int,int>> tmp;
        double d = dijkstraBase(grid, rx, ry, c.x, c.y, tmp);
        v.push_back(d);
    }
    return v;
}

//-----------------------------------------
// ESCOLHER CLUSTER A E B
//-----------------------------------------
std::pair<int,int> escolherClusterPar(
    const std::vector<std::vector<Cell>>& grid,
    int rx, int ry,
    const std::vector<Centroide>& centroides)
{
    if (centroides.size() < 2) return {-1,-1};

    auto dist = calcularDistanciasRoboParaCentroides(grid, rx, ry, centroides);

    int idxA = -1;
    double best = 1e18;

    for (int i = 0; i < dist.size(); i++) {
        if (dist[i] > 0 && dist[i] < best) {
            best = dist[i];
            idxA = i;
        }
    }

    if (idxA < 0) return {-1,-1};

    int Ax = centroides[idxA].x;
    int Ay = centroides[idxA].y;

    int idxB = -1;
    best = 1e18;

    for (int i = 0; i < centroides.size(); i++) {
        if (i == idxA) continue;

        std::vector<std::pair<int,int>> tmp;
        double d = dijkstraBase(grid, Ax, Ay, centroides[i].x, centroides[i].y, tmp);

        if (d > 0 && d < best) {
            best = d;
            idxB = i;
        }
    }

    return {idxA, idxB};
}

//-----------------------------------------
// GERAR CAMINHO ROBO → A  E  A → B
//-----------------------------------------
bool gerarCaminhoRoboAEB(
    const std::vector<std::vector<Cell>>& grid,
    int rx, int ry,
    const std::vector<Centroide>& centroides)
{
    caminhoRobo.clear();
    caminhoClusterAB.clear();
    caminhoCompleto.clear();

    auto [iA, iB] = escolherClusterPar(grid, rx, ry, centroides);
    if (iA < 0 || iB < 0) return false;

    int Ax = centroides[iA].x;
    int Ay = centroides[iA].y;
    int Bx = centroides[iB].x;
    int By = centroides[iB].y;

    // robo → A
    if (dijkstraBase(grid, rx, ry, Ax, Ay, caminhoRobo) < 0)
        return false;

    // A → B
    if (dijkstraBase(grid, Ax, Ay, Bx, By, caminhoClusterAB) < 0)
        return false;

    // caminho completo
    caminhoCompleto = caminhoRobo;
    caminhoCompleto.insert(
        caminhoCompleto.end(),
        caminhoClusterAB.begin()+1,  // evita duplicar A
        caminhoClusterAB.end()
    );

    return true;
}

//


inline double heurOctile(int x, int y, int gx, int gy) {
    double dx = std::abs(x - gx);
    double dy = std::abs(y - gy);
    return (dx + dy) + (std::sqrt(2.0) - 2.0) * std::min(dx, dy);
}
double aStarBase(
    const std::vector<std::vector<Cell>>& grid,
    int sx, int sy,
    int gx, int gy,
    std::vector<std::pair<int,int>>& outPath)
{
    const int H = grid.size();
    if (H == 0) return -1;
    const int W = grid[0].size();
    if (W == 0) return -1;

    auto inside = [&](int y, int x){
        return x >= 0 && x < W && y >= 0 && y < H;
    };

    const double INF = 1e18;

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
                COST_FREE,
                COST_UNKNOWN,   // unknown MUITO barato → explora desconhecido
                COST_OCCUPIED,
                COST_INFLATED,
                3
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
    const std::vector<Centroide>& cs)
{
    std::vector<double> v;
    v.reserve(cs.size());

    for (auto& c : cs) {
        std::vector<std::pair<int,int>> tmp;
        double d = aStarBase(grid, rx, ry, c.x, c.y, tmp);
        v.push_back(d);
    }
    return v;
}
std::pair<int,int> escolherABeBAstar(
    const std::vector<std::vector<Cell>>& grid,
    int rx, int ry,
    const std::vector<Centroide>& cs)
{
    if (cs.size() < 2) return {-1,-1};

    auto dR = distRoboParaCentroidesAstar(grid, rx, ry, cs);

    int idxA = -1;
    double best = 1e18;

    for (int i = 0; i < (int)cs.size(); i++) {
        if (dR[i] > 0 && dR[i] < best) {
            best = dR[i];
            idxA = i;
        }
    }
    if (idxA < 0) return {-1,-1};

    int Ax = cs[idxA].x;
    int Ay = cs[idxA].y;

    int idxB = -1;
    best = 1e18;

    for (int i = 0; i < (int)cs.size(); i++) {
        if (i == idxA) continue;

        std::vector<std::pair<int,int>> tmp;
        double d = aStarBase(grid, Ax, Ay, cs[i].x, cs[i].y, tmp);

        if (d > 0 && d < best) {
            best = d;
            idxB = i;
        }
    }

    return {idxA, idxB};
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


void* pathingThreadFunction(void* arg) {

    while (rclcpp::ok()) {
        std::cout << "Iniciou \n";

        if (!matrizMundo.empty() && !occGrid.empty()){
            std::cout << "Entrou \n";

            MatrixPosition matPosRobo = findCell(
                                            roboPosicao.x * scaleFactor - offset[0], 
                                            roboPosicao.y * scaleFactor - offset[1],        
                                            grid.inicio, 
                                            grid.passo
                                        );
            
            int linhas = matrizMundo.size();
            int colunas = matrizMundo[0].size();
            int chk = 0;
            
            if (posicaoValida(matPosRobo, linhas, colunas)){
                std::cout << "Checkpoint " << ++chk << "\n";
                        
                listaPontos = gerarPontos(matrizMundo);
                std::cout << "Checkpoint " << ++chk << "\n";

                if (!listaPontos.empty()){
                    std::cout << "Checkpoint " << ++chk << "\n";

                    detectarFronteiras(listaPontos);
                    std::cout << "Checkpoint " << ++chk << "\n";
                    std::vector<Ponto> fronteiras;
                    for (auto& p : listaPontos)
                        if (p.isFrontier)
                            fronteiras.push_back(p);

                    rodarDBSCAN(fronteiras, 5.0f, 10); 
                    std::cout << "Checkpoint " << ++chk << "\n";

                    listaCentroides = calcularCentroides(fronteiras);
                    std::cout << "Checkpoint " << ++chk << "\n";

                    calcularDistanciasVizinho(listaCentroides);
                    std::cout << "Checkpoint " << ++chk << "\n";

                    std::cout << "Centroides encontrados: " << listaCentroides.size() << "\n";

                    for (auto& c : listaCentroides) {
                        std::cout << "Cluster: " << c.clusterId << " (" << c.x << ", " << c.y << ") " << " Size: " << c.numPontos << " NN: " << c.distVizinho << "\n";
                    }
                    std::cout << "Checkpoint " << ++chk << "\n";

                    std::vector<double> distancias = calcularDistanciasRoboParaCentroides(occGrid, matPosRobo.coluna, matPosRobo.linha, listaCentroides);
                    for (size_t i = 0; i < distancias.size(); i++) {
                        std::cout << "Cluster " << listaCentroides[i].clusterId
                                << " | Distancia = " << distancias[i] << std::endl;
                    }   
                    std::cout << "Checkpoint " << ++chk << "\n";

                    gerarCaminhoAstarCompleto(occGrid, matPosRobo.coluna, matPosRobo.linha, listaCentroides, caminhoCompleto);
                    /*
                    if(
                        //gerarCaminhoRoboAEB(occGrid, matPosRobo.coluna, matPosRobo.linha, listaCentroides) &&
                        !caminhoRobo.empty() && 
                        !caminhoClusterAB.empty()
                    ){
                        std::cout << "Checkpoint " << ++chk << "\n";

                        caminhoCompleto.clear();
                        caminhoCompleto.reserve(caminhoRobo.size() + caminhoClusterAB.size());

                        // 1) Copiar caminho do Robô até A
                        for (const auto& p : caminhoRobo) {
                            caminhoCompleto.push_back(p);
                        }

                        // 2) Copiar caminho de A até B (sem repetir o ponto A)
                        for (size_t i = 1; i < caminhoClusterAB.size(); i++) {
                            caminhoCompleto.push_back(caminhoClusterAB[i]);
                        }

                    }*/
                }
            }
        }
        // Pequena pausa para não sobrecarregar a CPU
        usleep(1000000); // 1000ms
    }

    return NULL;
}