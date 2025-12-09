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
extern std::vector<float> sonares;
extern std::vector<float> laseres;
extern std::vector<std::vector<bool>> knownRegion;

std::vector<double> sensorAngles = {-90, -50, -30, -10, 10, 30, 50, 90, 90, 130, 150, 170, -170, -150, -130, -90};
std::vector<int> sensorIndices = {0, 1, 2, 3, 4, 5, 6, 7};//, 9, 10, 11, 12, 13, 14};
//std::vector<float> offset = {0.8, 0.7};
std::vector<float> offset = {0.0, 0.0};
float scaleFactor = 0.025f;

// Histórico de posições
std::vector<Position> caminho;

// Cria Grade e Matrizes
GridInfo grid = {-1.0f, 1.0f, 0.005f};
int size = (grid.fim - grid.inicio) / grid.passo;  
// Bayes
// std::vector<std::vector<float>> matrizMundo(size, std::vector<float>(size, 0.5f));
// HIMM
std::vector<std::vector<float>> matrizMundo(size, std::vector<float>(size, 7.5f));
std::vector<std::vector<int>> matrizPath(size, std::vector<int>(size, 0));
std::vector<std::vector<Cell>> occGrid(size, std::vector<Cell>(size, Cell{true, false, false}));


// Estrutura para lista de pontos
std::vector<Ponto> listaPontos;
std::vector<Centroide> listaCentroides;
std::vector<std::pair<int,int>> caminhoRobo;
std::vector<std::pair<int,int>> caminhoClusterAB;
std::vector<std::pair<int,int>> caminhoCompleto;


float round2(float valor) {
    return std::round(valor * 100.0f) / 100.0f;
}

void salvaMatriz(const std::vector<std::vector<float>>& matriz, const std::string& nomeArquivo) {
    std::ofstream arquivo(nomeArquivo);

    if (!arquivo.is_open()) {
        std::cerr << "Erro ao abrir o arquivo " << nomeArquivo << " para escrita." << std::endl;
        return;
    }

    for (const auto& linha : matriz) {
        for (const auto& valor : linha) {
            arquivo << valor << " ";
        }
        arquivo << "\n";
    }

    arquivo.close();
    std::cout << "Matriz salva com sucesso em " << nomeArquivo << std::endl;
}

std::vector<std::vector<float>> loadMatrix(const std::string& nomeArquivo) {
    std::ifstream arquivo(nomeArquivo);
    std::vector<std::vector<float>> matriz;

    if (!arquivo.is_open()) {
        std::cerr << "Erro ao abrir o arquivo " << nomeArquivo << " para leitura." << std::endl;
        return matriz;
    }

    std::string linha;
    while (std::getline(arquivo, linha)) {
        std::istringstream stream(linha);
        std::vector<float> linhaMatriz;
        float valor;

        while (stream >> valor) {
            linhaMatriz.push_back(valor);
        }

        if (!linhaMatriz.empty()) {
            matriz.push_back(linhaMatriz);
        }
    }

    arquivo.close();
    std::cout << "Matriz carregada com sucesso de " << nomeArquivo << std::endl;
    return matriz;
}

MatrixPosition findCell(float x, float y, float inicio, float passo) {
    int coluna = static_cast<int>((x - inicio) / passo);
    int linha  = static_cast<int>((y - inicio) / passo);

    return {linha, coluna};
} // Conversão de ponto para célula da matriz

CellCenter centroDaCelula(const MatrixPosition& pos, float inicio, float passo) {
    float x = inicio + pos.coluna * passo + passo / 2.0f;
    float y = inicio + pos.linha  * passo + passo / 2.0f;
    return {x, y};
} // Ponto central da célula

CellRelativeInfo calculaDistanciaEAngulo(const CellCenter& centroRobo,const CellCenter& centroPonto,float yawRobo) {
    float dx = centroPonto.x - centroRobo.x;
    float dy = centroPonto.y - centroRobo.y;

    float distancia = std::sqrt(dx * dx + dy * dy);
    float anguloAbsoluto = std::atan2(dy, dx);

    // Normaliza para [0, 2PI)
    if(anguloAbsoluto < 0){
        anguloAbsoluto += 2 * M_PI;
    }

    float yawRad = yawRobo;
    if (yawRad < 0) yawRad += 2 * M_PI;

    float anguloRelativo = anguloAbsoluto - yawRad;

    // Normaliza para [-PI, PI]
    if (anguloRelativo > M_PI) anguloRelativo -= 2 * M_PI;
    if (anguloRelativo < -M_PI) anguloRelativo += 2 * M_PI;

    // Converte para graus
    float anguloRelativoGraus = anguloRelativo * 180.0f / M_PI;

    return {distancia, anguloRelativoGraus};
} // Distancia e angulo entre dois pontos (centro de celulas)

bool posicaoValida(const MatrixPosition& pos, int linhas, int colunas) {
    return (pos.linha >= 0 && pos.linha < linhas &&
            pos.coluna >= 0 && pos.coluna < colunas);
} // Verifica se a posição faz parte da matriz 

float bayes(float R, float r, float s, float beta, float alpha, float max, float pOcup) {

    float rangeDetect = 0.01f;
    float pOcupS = 0.5f;

    if ((r >= s - rangeDetect) && (r <= s + rangeDetect) && (s <= R)){
        float pSOcup = 0.5f * ( (R-r)/R + (beta-std::abs(alpha))/beta ) * max;
        float pSVaz = 1.0f - pSOcup;
        float pSOcup_pOcup = pSOcup * pOcup;
        float pSVaz_pVaz = pSVaz * ( 1 - pOcup );
        pOcupS = (pSOcup_pOcup / (pSOcup_pOcup + pSVaz_pVaz));
    }
    else{
        float pVaz = 1 - pOcup;
        float pSVaz = 0.5f * ( (R-r)/R + (beta-std::abs(alpha))/beta ) * max;
        float pSOcup = 1.0f - pSVaz;
        float pSVaz_pVaz = pSVaz * pVaz;
        float pSOcup_pOcup = pSOcup * pOcup;

        pOcupS = (pSOcup_pOcup / (pSOcup_pOcup + pSVaz_pVaz));
    }

    // Debug 
    /*if (false){
        std::cout 
            << "pSOcup: " << pSOcup << " "
            << "pSVaz: " << pSVaz << " "
            << "pOcup: " << pOcup << " "
            << "pSOcup_pOcup: " << pSOcup_pOcup << " "
            << "pSVaz_pVaz: " << pSVaz_pVaz << " "
            << "pOcupS: " << pOcupS << " "
        << std::endl;
    }*/

    return pOcupS;

} // Calculo da probabilidade de ocupação por bayes

void atualizaMatrizBayes(std::vector<std::vector<float>>& matriz, Robot robot, float sensorAngle) {
    int linhas = matriz.size();
    int colunas = matriz[0].size();

    for (int i = 0; i < linhas; ++i) {
        for (int j = 0; j < colunas; ++j) {
            
            MatrixPosition posAtual = {i, j};
            CellCenter centroAtual = centroDaCelula(posAtual, grid.inicio, grid.passo);

            // Calcula ângulo relativo e distância entre célula e centro do robô
            CellRelativeInfo relations = calculaDistanciaEAngulo(
                robot.cellCenter, 
                centroAtual, 
                robot.pos.theta - sensorAngle // yaw do robô + ângulo do sensor
            );

            float r = relations.distancia; // em relação ao mapa
            float alpha = relations.anguloRelativo; // em relação ao mapa
            float beta = robot.beta;
            float delta = 0.01f;
            float R = 2.0f; // alcance máximo do sensor que considero válido

            // Está no cone do sensor?
            if (r <= R * scaleFactor && (r <= robot.s + delta) && alpha >= -beta && alpha <= beta) {
                float pOcup_atual = matriz[i][j];
                float pOcup = bayes(R, r, robot.s, beta, alpha, 0.98f, pOcup_atual);
                matriz[i][j] = pOcup;
            }
        }
    }

    //salvaMatriz(matriz, "matriz.txt");
} // Método de atualização das células da matriz para ocupação

std::vector<MatrixPosition> bresenham(MatrixPosition start, MatrixPosition end) {
    std::vector<MatrixPosition> points;

    int x0 = start.coluna;
    int y0 = start.linha;
    int x1 = end.coluna;
    int y1 = end.linha;

    int dx = abs(x1 - x0);
    int dy = -abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    while (true) {
        points.push_back({y0, x0});

        if (x0 == x1 && y0 == y1) break;

        int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }

    return points;
}

void atualizaMatrizHIMM(
    std::vector<std::vector<float>>& matriz,
    const Robot& robot,
    float sensorAngle,
    float maxRange = 2.0f,
    float incremento = 3.0f,
    float decremento = 1.0f,
    float minValor = 0.0f,
    float maxValor = 15.0f
) {
    // Calcula a posição final do feixe do sensor
    float anguloGlobal = robot.pos.theta - sensorAngle;
    bool noDetect = false;

    float distanciaSensor = robot.s;
    if (distanciaSensor > maxRange*scaleFactor) {
        distanciaSensor = maxRange*scaleFactor;
        noDetect = true;
    }

    float xFinal = robot.cellCenter.x + cos(anguloGlobal) * distanciaSensor;
    float yFinal = robot.cellCenter.y + sin(anguloGlobal) * distanciaSensor;

    // Converte posições reais em posições de célula
    MatrixPosition celulaInicial = robot.gridPos;
    MatrixPosition celulaFinal = findCell(xFinal, yFinal, grid.inicio, grid.passo);
    if (!posicaoValida(celulaFinal, matriz.size(), matriz[0].size())) return;

    // Algoritmo de Bresenham para traçar o feixe entre celulaInicial e celulaFinal
    std::vector<MatrixPosition> caminho = bresenham(celulaInicial, celulaFinal);

    // Marca as células atravessadas como livres
    for (size_t i = 0; i + 1 < caminho.size(); ++i) { // exceto a última
        float& cell = matriz[caminho[i].linha][caminho[i].coluna];
        cell = std::max(minValor, cell - decremento);
        knownRegion[caminho[i].linha][caminho[i].coluna] = true;
    }

    // Marca a última célula (possível obstáculo) como ocupada
    if (!caminho.empty()) {
        MatrixPosition ocupada = caminho.back();
        float& cellCentral = matriz[ocupada.linha][ocupada.coluna];

        if (!noDetect) {
            float soma = 0.0f;
            float pesos[3][3] = {
                {0.5f, 0.5f, 0.5f},
                {0.5f, 1.0f, 0.5f},
                {0.5f, 0.5f, 0.5f}
            };

            for (int i = -1; i <= 1; ++i) {
                for (int j = -1; j <= 1; ++j) {
                    int linhaVizinha = ocupada.linha + i;
                    int colunaVizinha = ocupada.coluna + j;

                    if (posicaoValida({linhaVizinha, colunaVizinha}, matriz.size(), matriz[0].size())) {
                        float peso = pesos[i + 1][j + 1];

                        if (i == 0 && j == 0) {
                            soma += incremento * peso;  // centro usa o incremento
                        } else {
                            soma += matriz[linhaVizinha][colunaVizinha] * peso; // vizinhos usam valor atual
                        }
                    }
                }
            }

            cellCentral = std::min(maxValor, soma);
        } else {
            cellCentral = std::max(minValor, cellCentral - decremento);
        }

        knownRegion[ocupada.linha][ocupada.coluna] = true;
    }
}// Método de atualização das células da matriz para ocupação

void montarOccGrid(
    const std::vector<std::vector<float>>& matA,
    const std::vector<std::vector<bool>>& matB,
    std::vector<std::vector<Cell>>& matC
) {
    int linhas = matA.size();
    int colunas = matA[0].size();

    matC.assign(linhas, std::vector<Cell>(colunas));

    for (int y = 0; y < linhas; y++) {
        for (int x = 0; x < colunas; x++) {

            // Se matB é false, célula é Unknown
            if (!matB[y][x]) {
                matC[y][x].isUnknown = true;
                continue;
            }

            float v = matA[y][x];

            if (v >= 0 && v <= 7) {
                matC[y][x].isFree = true;
            }
            else if (v >= 8 && v <= 15) {
                matC[y][x].isOcc = true;
            }
            else {
                // Caso inesperado → unknown
                matC[y][x].isUnknown = true;
            }
        }
    }
}

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
    if (matrizMundo.empty()) return pontos;

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


// djikstra


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

            if (grid[ny][nx].isOcc) continue;

            double mc = (k < 4 ? 1.0 : sqrt(2.0));
            double w  = (grid[ny][nx].isUnknown ? 1.0 : 10.0);

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

void* mappingThreadFunction(void* arg) {
    while (rclcpp::ok()) {
        MatrixPosition matPosRobo = findCell(roboPosicao.x * scaleFactor - offset[0], 
                                              roboPosicao.y * scaleFactor - offset[1], 
                                              grid.inicio, grid.passo);
        
        int linhas = matrizMundo.size();
        int colunas = matrizMundo[0].size();

        if (posicaoValida(matPosRobo, linhas, colunas) && !laseres.empty()) {

            CellCenter centroCelRobo = centroDaCelula(matPosRobo, grid.inicio, grid.passo);
            Robot robotInfo = {matPosRobo, roboPosicao, centroCelRobo, 0.0f};

            
            for (int idx = 0; idx <= 30; idx++) {
                robotInfo.s = laseres[6*idx] * scaleFactor;
                float sensorAngle = (6*idx - 90) * M_PI / 180.0f;
                atualizaMatrizHIMM(matrizMundo, robotInfo, sensorAngle);
            }
            
        }

        montarOccGrid(matrizMundo, knownRegion, occGrid);

        listaPontos = gerarPontos(matrizMundo);
        detectarFronteiras(listaPontos);

        std::vector<Ponto> fronteiras;
        for (auto& p : listaPontos)
            if (p.isFrontier)
                fronteiras.push_back(p);

        rodarDBSCAN(fronteiras, 3.0f, 5); // eps, minPts

        listaCentroides = calcularCentroides(fronteiras);
        calcularDistanciasVizinho(listaCentroides);

        std::cout << "Centroides encontrados: " << listaCentroides.size() << "\n";

        for (auto& c : listaCentroides) {
            std::cout << "Cluster: " << c.clusterId << " (" << c.x << ", " << c.y << ") " << " Size: " << c.numPontos << " NN: " << c.distVizinho << "\n";
        }

        std::vector<double> distancias = calcularDistanciasRoboParaCentroides(occGrid, matPosRobo.coluna, matPosRobo.linha, listaCentroides);
        for (size_t i = 0; i < distancias.size(); i++) {
            std::cout << "Cluster " << listaCentroides[i].clusterId
                    << " | Distancia = " << distancias[i] << std::endl;
        }   

        //gerarCaminhoRoboAEB(occGrid, matPosRobo.coluna, matPosRobo.linha, listaCentroides);

        //if(!caminhoRobo.empty() & !caminhoClusterAB.empty()){
        if(
            gerarCaminhoRoboAEB(occGrid, matPosRobo.coluna, matPosRobo.linha, listaCentroides) &
            !caminhoRobo.empty() & 
            !caminhoClusterAB.empty()
        ){

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

        }

        // Pequena pausa para não sobrecarregar a CPU
        usleep(10000); // 10ms
    }

    return NULL;
}