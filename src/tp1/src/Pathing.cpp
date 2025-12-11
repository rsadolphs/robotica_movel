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
#include <mutex>
#include <atomic>

std::mutex caminhoMutex;
std::atomic<int> caminhoVersion{0};

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

const int COST_DIFF = 20;
const int INFL_RADIUS = 3;
const double HISTERESE = 0.85;

double cost_free = 1.0;
double cost_unk = 1.0;
double cost_occ = 1e5;

int nClusters = -1;  // Inicia com -1
bool startedClustering = false;


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

inline double custoCelulaComInflacaoGradual(
    const std::vector<std::vector<Cell>>& grid,
    int y, int x,
    double cost_free,
    double cost_unk,
    double cost_occ,
    int inflationRadius = 3)
{
    double penalidade = 0.0;

    if (grid[y][x].isOcc){
        return cost_occ;  // obstáculo direto permanece muito alto
    }

    for (int dy = -inflationRadius; dy <= inflationRadius; dy++) {
        for (int dx = -inflationRadius; dx <= inflationRadius; dx++) {
            if (dx == 0 && dy == 0) continue;

            int ny = y + dy;
            int nx = x + dx;

            if (ny < 0 || nx < 0 || ny >= grid.size() || nx >= grid[0].size())
                continue;

            if (grid[ny][nx].isOcc) {
                int dist = std::max(std::abs(dx), std::abs(dy));
                if (dist == 1) penalidade += 3.0 * COST_DIFF;
                else if (dist == 2) penalidade += 2.0 * COST_DIFF;
                else if (dist == 3) penalidade += 1.0 * COST_DIFF;
            }
        }
    }

    if (grid[y][x].isUnknown){
        return cost_unk + penalidade;
    }
    else{
        return cost_free + penalidade;
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
    int sx, int sy,          // start x,y
    int gx, int gy,          // goal x,y
    std::vector<std::pair<int,int>>& outPath,
    bool globalMode
){
    // --------------------------------------
    // Validação da grade
    // --------------------------------------
    const int H = grid.size();
    if (H == 0) return -1;

    const int W = grid[0].size();
    if (W == 0) return -1;

    // --------------------------------------
    // Parâmetros de custo (global/local)
    // --------------------------------------
    if (globalMode) {
        cost_free = COST_DIFF;   // custo de célula livre
        cost_unk  = 1.0;         // custo de célula desconhecida
    }
    else {
        cost_free = 1.0;
        cost_unk  = COST_DIFF * 100;
    }
    // cost_occ é global

    const double INF = 1e9;

    // --------------------------------------
    // Função auxiliar: dentro do mapa
    // --------------------------------------
    auto inside = [&](int y, int x){
        return (x >= 0 && x < W && y >= 0 && y < H);
    };

    // --------------------------------------
    // Buffers do A*
    // --------------------------------------
    std::vector<std::vector<double>> g(H, std::vector<double>(W, INF));
    std::vector<std::vector<std::pair<int,int>>> parent(
        H, std::vector<std::pair<int,int>>(W, {-1, -1})
    );

    using Node = std::tuple<double,double,int,int>; 
    // Node = (f, g, y, x)

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;

    // --------------------------------------
    // Inicialização
    // --------------------------------------
    g[sy][sx] = 0.0;
    pq.push({ heurOctile(sx, sy, gx, gy), 0.0, sy, sx });

    // Movimentos 8-conectados
    const int dx[8] = { 1,-1,0,0,  1,1,-1,-1 };
    const int dy[8] = { 0,0,1,-1,  1,-1, 1,-1 };

    // --------------------------------------
    // Loop principal do A*
    // --------------------------------------
    while (!pq.empty()) {

        auto [f, gc, y, x] = pq.top();
        pq.pop();

        // Se este nó já não é ótimo, ignore
        if (gc > g[y][x]) continue;

        // --------------------------------------
        // Encontrou o objetivo → reconstrói caminho
        // --------------------------------------
        if (x == gx && y == gy) {
            outPath.clear();

            int cy = gy, cx = gx;
            while (!(cy == sy && cx == sx)) {
                outPath.emplace_back(cy, cx);
                auto [py, px] = parent[cy][cx];
                cy = py;
                cx = px;
            }
            outPath.emplace_back(sy, sx);
            std::reverse(outPath.begin(), outPath.end());

            return g[gy][gx];
        }

        // --------------------------------------
        // Explora vizinhos
        // --------------------------------------
        for (int k = 0; k < 8; k++) {

            int nx = x + dx[k];
            int ny = y + dy[k];

            if (!inside(ny, nx)) continue;

            // Custo base do movimento
            double stepCost = (k < 4 ? 1.0 : std::sqrt(2.0));

            // Custo da célula com inflação
            double w = custoCelulaComInflacaoGradual(
                grid,
                ny, nx,
                cost_free,
                cost_unk,
                cost_occ,
                INFL_RADIUS
            );

            double ng = gc + stepCost * w;

            // Se encontramos um caminho melhor para esta célula
            if (ng < g[ny][nx]) {
                g[ny][nx] = ng;
                parent[ny][nx] = {y, x};

                double nf = ng + heurOctile(nx, ny, gx, gy);
                pq.push({nf, ng, ny, nx});
            }
        }
    }

    // Objetivo não encontrado
    return -1;
}


//=====================================================================================
double pathCost(const std::vector<std::pair<int,int>>& p) {
    return (double)p.size();
}

std::vector<CaminhoInfo> calcularCustoRA(
    const std::vector<std::vector<Cell>>& grid,
    int rx, int ry,
    const std::vector<Centroide>& cs)
{
    std::vector<CaminhoInfo> custoRA(cs.size());

    for (int i = 0; i < (int)cs.size(); i++) {

        int ax = cs[i].x;   // centroide X
        int ay = cs[i].y;   // centroide Y

        std::vector<std::pair<int,int>> path;
        double d = aStarBase(grid, rx, ry, ax, ay, path, /*globalMode=*/false);

        if (d >= 0 && !path.empty()) {
            custoRA[i].custo  = d;
            custoRA[i].path   = std::move(path);  
            custoRA[i].valido = true;
        }
    }
    return custoRA;
}

std::vector<std::vector<CaminhoInfo>> calcularCustoAB(
    const std::vector<std::vector<Cell>>& grid,
    const std::vector<Centroide>& cs)
{
    int n = cs.size();
    std::vector<std::vector<CaminhoInfo>> custoAB(
        n, std::vector<CaminhoInfo>(n));

    for (int i = 0; i < n; i++) {

        int ax = cs[i].x;
        int ay = cs[i].y;

        for (int j = 0; j < n; j++) {
            if (i == j) continue;

            int bx = cs[j].x;
            int by = cs[j].y;

            std::vector<std::pair<int,int>> path;
            double d = aStarBase(grid, ax, ay, bx, by, path, /*globalMode=*/false);

            if (d >= 0 && !path.empty()) {
                custoAB[i][j].custo  = d;
                custoAB[i][j].path   = std::move(path);
                custoAB[i][j].valido = true;
            }
        }
    }
    return custoAB;
}


// Escolhe A e B
std::pair<int,int> escolherAB(
    const std::vector<std::vector<Cell>>& grid,
    int rx, int ry,
    const std::vector<Centroide>& cs)
{
    int n = cs.size();
    if (n < 2) return {-1, -1};

    // 1) custo R->A
    auto custoRA = calcularCustoRA(grid, rx, ry, cs);

    // 2) custo A->B
    auto custoAB = calcularCustoAB(grid, cs);

    // 3) busca melhor combinação
    double bestCost = 1e18;
    int bestA = -1, bestB = -1;

    for (int i = 0; i < n; i++) {
        if (!custoRA[i].valido) continue;

        for (int j = 0; j < n; j++) {
            if (i == j) continue;
            if (!custoAB[i][j].valido) continue;

            double total = custoRA[i].custo + custoAB[i][j].custo;
            if (total < bestCost) {
                bestCost = total;
                bestA = i;
                bestB = j;
            }
        }
    }

    return {bestA, bestB};
}
//=====================================================================================

bool gerarCaminhoAstarCompleto(
    const std::vector<std::vector<Cell>>& grid,
    int rx, int ry,
    const std::vector<Centroide>& cs,
    std::vector<std::pair<int,int>>& outFinal)
{
    outFinal.clear();

    //auto [iA, iB] = escolherABeBAstar(grid, rx, ry, cs);
    auto [iA, iB] = escolherAB(grid, rx, ry, cs);
    if (iA < 0 || iB < 0) return false;

    int Ax = cs[iA].x;
    int Ay = cs[iA].y;
    int Bx = cs[iB].x;
    int By = cs[iB].y;

    std::vector<std::pair<int,int>> pathR;
    std::vector<std::pair<int,int>> pathAB;

    if (aStarBase(grid, rx, ry, Ax, Ay, pathR, false) < 0)
        return false;

    if (aStarBase(grid, Ax, Ay, Bx, By, pathAB, true) < 0)
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

static MatrixPosition lastPos = {-1, -1};

void* pathingThreadFunction(void* arg) {

    while (rclcpp::ok()) {

        if (!matrizMundo.empty() && !occGrid.empty()) {

            MatrixPosition matPosRobo = findCell(
                roboPosicao.x * scaleFactor - offset[0],
                roboPosicao.y * scaleFactor - offset[1],
                grid.inicio,
                grid.passo
            );

            int linhas = matrizMundo.size();
            int colunas = matrizMundo[0].size();

            if (posicaoValida(matPosRobo, linhas, colunas)) {

                listaPontos = gerarPontos(matrizMundo);
                if (listaPontos.empty()) {
                    usleep(100000);
                    continue;
                }

                // --- Detectar fronteiras e clusters ---
                detectarFronteiras(listaPontos);

                std::vector<Ponto> fronteiras;
                for (auto& p : listaPontos)
                    if (p.isFrontier)
                        fronteiras.push_back(p);

                rodarDBSCAN(fronteiras, 5.0f, 10);
                listaCentroides = calcularCentroides(fronteiras);

                if (listaCentroides.empty()) {
                    usleep(100000);
                    continue;
                }

                calcularDistanciasVizinho(listaCentroides);
                nClusters = listaCentroides.size();

                // -----------------------------------------------------
                //  🔥 SÓ GERA CAMINHO A* SE O ROBÔ MUDOU DE CÉLULA !!!
                // -----------------------------------------------------
                if (matPosRobo.linha != lastPos.linha ||
                    matPosRobo.coluna != lastPos.coluna)
                {
                    // gera em vetor local como você já faz
                    std::vector<std::pair<int,int>> novoCaminho;
                    bool ok = gerarCaminhoAstarCompleto(
                        occGrid,
                        matPosRobo.coluna,
                        matPosRobo.linha,
                        listaCentroides,
                        novoCaminho
                    );

                    // debug básico sobre o que veio
                    std::cout << "[PATHING] gerarCaminho returned ok=" << ok 
                            << " novo.size=" << novoCaminho.size() << std::endl;

                    if (ok && !novoCaminho.empty()) {

                        // 1) remover pontos consecutivos idênticos (ruídos)
                        std::vector<std::pair<int,int>> compact;
                        compact.reserve(novoCaminho.size());
                        for (size_t i = 0; i < novoCaminho.size(); ++i) {
                            if (i == 0 || novoCaminho[i] != novoCaminho[i-1])
                                compact.push_back(novoCaminho[i]);
                        }
                        novoCaminho.swap(compact);

                        // 2) remover prefixo duplicado simples
                        // Se o caminho contém A||A (prefixo A repetido duas vezes), mantém só a última ocorrência
                        auto N = novoCaminho.size();
                        for (size_t len = 1; len*2 <= N; ++len) {
                            bool prefixEqualsSuffix = true;
                            for (size_t k = 0; k < len; ++k) {
                                if (novoCaminho[k] != novoCaminho[k+len]) {
                                    prefixEqualsSuffix = false;
                                    break;
                                }
                            }
                            if (prefixEqualsSuffix) {
                                // mantém apenas segunda metade
                                std::vector<std::pair<int,int>> reduced(novoCaminho.begin()+len, novoCaminho.end());
                                novoCaminho.swap(reduced);
                                break;
                            }
                        }

                        // Debug dos extremos
                        if (!novoCaminho.empty()) {
                            auto p0 = novoCaminho.front();
                            auto pN = novoCaminho.back();
                            std::cout << "[PATHING] novoCaminho first=(" << p0.first << "," << p0.second 
                                    << ") last=(" << pN.first << "," << pN.second << ") size=" << novoCaminho.size() << std::endl;
                        }

                        // 3) mover para o global ATOMICAMENTE e incrementar versão
                        {
                            std::lock_guard<std::mutex> lk(caminhoMutex);
                            caminhoCompleto = std::move(novoCaminho);
                            caminhoVersion.fetch_add(1, std::memory_order_relaxed);
                        }
                    }

                }


                // --- Cálculo de yaw (não gera caminho) ---
                yawAstar = calcularYawParaCaminho(caminhoCompleto, matPosRobo);

                startedClustering = true;
            }
        }

        usleep(100000); // 100ms
    }

    return NULL;
}
