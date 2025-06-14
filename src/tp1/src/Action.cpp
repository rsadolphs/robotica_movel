#include "Action.h"
#include "Utils.h"
#include "graphics.hpp"
#include "Mapping.hpp"
#include "PotentialField.hpp"

#include <vector>
#include <iostream>
#include <limits>
#include <cmath>
#include <tuple>
#include <array>

extern std::vector<std::vector<float>> matrizMundo;  // mapping.cpp
extern std::vector<std::vector<float>> campoPotencial;  // PotentialField.cpp
extern GridInfo grid;

Position roboPosicao = {0.0f, 0.0f, 0.0f}; 
MovingDirection side;
extern float scaleFactor;

int firstMinDistPos = 0;
bool firstInfo = false;
std::vector<Position> positionArray;
std::vector<float> sonares;

extern std::vector<double> sensorAngles;

std::vector<std::vector<int>> senseWalls(const std::vector<float>& sonars) { 

    std::vector<std::vector<int>> knownSpace(100, std::vector<int>(100, 0));
    std::vector<std::tuple<int, int>> capturedPoints;

    for (int i = 0; i < sonars.size(); ++i) {

        float reading = sonars[i] * 10.0;
        float angleRad = sensorAngles[i] * M_PI / 180.0;

        int dx = static_cast<int>(std::round(reading * std::cos(angleRad)));
        int dy = static_cast<int>(std::round(reading * std::sin(angleRad)));

        int x = 50 + dx;
        int y = 50 + dy;

        if (x<0){x=0;}
        if (y<0){y=0;}

        std::cout << "COORD " << i << ": (" << x << "," << y << ")" << std::endl;
        capturedPoints.emplace_back(x, y);

        if (x >= 0 && x < 100 && y >= 0 && y < 100) {
            knownSpace[x][y] = 1;
        }
    } 
    
    for (int i = 0; i < capturedPoints.size(); ++i) {

        int a = i + 1;
        int b = i + 2;

        if (i + 2 == capturedPoints.size()){ 
            b = 0;
        }
        else{
            if (i + 1 == capturedPoints.size()){ 
                a = 0; 
                b = 1;
            }
        }

        std::array<std::array<int, 3>, 3> mat = {{
            {std::get<0>(capturedPoints[i]), std::get<1>(capturedPoints[i]), 1},
            {std::get<0>(capturedPoints[a]), std::get<1>(capturedPoints[a]), 1},
            {std::get<0>(capturedPoints[b]), std::get<1>(capturedPoints[b]), 1}
        }};

        int det = mat[0][0]*(mat[1][1]*mat[2][2] - mat[1][2]*mat[2][1])
            - mat[0][1]*(mat[1][0]*mat[2][2] - mat[1][2]*mat[2][0])
            + mat[0][2]*(mat[1][0]*mat[2][1] - mat[1][1]*mat[2][0]);

        
        std::cout << "Det (" << i << "," << a << "," << b << ") : " << det << std::endl;

    }

    return knownSpace;
}

std::pair<int, float> findMinPosition(const std::vector<float>& vec) {
    // Verifica se o vetor está vazio
    if (vec.empty()) {
        std::cerr << "O vetor está vazio!" << std::endl;
        return {-1, 0.0f};  // Retorna posição inválida e valor 0 se vetor vazio
    }

    int minPosition = 0;  // Inicializa a posição com o primeiro índice
    float minValue = vec[0];  // Inicializa o menor valor com o primeiro elemento

    // Percorre o vetor
    for (size_t i = 1; i < vec.size(); ++i) {
        if (vec[i] < minValue) {
            minValue = vec[i];  // Atualiza o menor valor
            minPosition = i;    // Atualiza a posição do menor valor
        }
    }

    return {minPosition, minValue};  // Retorna a posição e o valor do menor elemento
}

float yawGradiente(
    const std::vector<std::vector<float>>& campoPotencial,
    float posX, float posY
) {
    // Verifica se estamos em uma posição válida (não na borda)
    int largura = campoPotencial[0].size();
    int altura  = campoPotencial.size();
    
    MatrixPosition matPos = findCell(posX, posY, grid.inicio, grid.passo);

    int x = matPos.coluna;
    int y = matPos.linha;

    if (x <= 0 || x >= largura - 1 || y <= 0 || y >= altura - 1) {
        // Fora da área onde dá pra calcular o gradiente central
        std::cout << "FORA DA AREA DE MAPEAMENTO" << std::endl;
        std::cout << "SIZE: " << largura << "," << altura << std::endl;
        std::cout << "POS: " << x << "," << y << std::endl;
        return 0.0f;
    }

    // Gradiente com diferenças centrais
    float dx = campoPotencial[y][x + 1] - campoPotencial[y][x - 1];
    float dy = campoPotencial[y + 1][x] - campoPotencial[y - 1][x];

    // Direção do declive (gradiente descendente)
    float yaw = std::atan2(-dy, -dx); // negativo pois queremos a direção da descida

    return yaw; // em radianos
}

Controle controleRobo(
    float yawAtual,
    float yawAlvo,
    PID& pid,
    float dt = 0.05f,       // tempo entre atualizações (ex: 20 Hz)
    float tolerancia = 0.05f // erro aceitável em radianos (0.1 = ~5.7 graus)
) {
    Controle ctrl;

    // Calcula erro angular (normaliza para intervalo [-PI, PI])
    float erro = yawAlvo - yawAtual;
    while (erro > M_PI) erro -= 2 * M_PI;
    while (erro < -M_PI) erro += 2 * M_PI;

    // PID (pode ser simplificado só com P se quiser)
    pid.erroAcumulado += erro * dt;
    float derivada = (erro - pid.erroAnterior) / dt;
    pid.erroAnterior = erro;

    ctrl.angVel = pid.kp * erro + pid.ki * pid.erroAcumulado + pid.kd * derivada;

    // Se o robô já está bem alinhado, move para frente
    if (std::abs(erro) < tolerancia) {
        ctrl.linVel = 0.5f;  // velocidade linear positiva (ajustável)
    } else {
        ctrl.linVel = 0.1f;  // parado enquanto gira
    }

    return ctrl;
}


Action::Action()
{
    linVel = 0.0;
    angVel = 0.0;
}

void Action::avoidObstacles(std::vector<float> lasers, std::vector<float> sonars, std::vector<float> pose)
{

    roboPosicao = {pose[0], pose[1], pose[2]};
    sonares = sonars;
    positionArray.push_back(roboPosicao);

    auto [minPos, minDist] = findMinPosition(sonars);  // retorna o sensor com menor distancia e o valor.

    if (minPos>=0 && minPos <=7 && minDist <= 1){
        if (minPos <= 3){
            linVel= 0.0; angVel=-1.0; // rotação em sentido horário
        }
        else{
            linVel= 0.0; angVel=1.0; // rotação em sentido anti horário
        }
    }
    else{
        linVel= 3.0; angVel= 0.0; // movimenta pra frente
    }


}

void Action::keepAsFarthestAsPossibleFromWalls(std::vector<float> lasers, std::vector<float> sonars)
{
    // TODO: COMPLETAR FUNCAO
    /*
    float minDistance = 0.0f;
    int minDistPos = 0;

    auto [minPos, minDist] = findMinPosition(sonars); 


    if (minPos > 7 && minDist >= 1){
        if (sonars[0] < sonars[7]){
            linVel=1.0; angVel=-0.5;
        }
        else{
            if (sonars[7] < sonars[0]){
                linVel=1.0; angVel=0.5;
            }
            else{
                linVel=1.0; angVel=0.0;
            }
        }
    }
    else{
        if (minPos <= 3){
            linVel= 0.0; angVel=-1.0; // rotação em sentido horário
        }
        else{
            linVel= 0.0; angVel=1.0; // rotação em sentido anti horário
        }
        
    }
    */

    auto matrix = senseWalls(sonars);

}

float distPontos(Position p1, Position p2){
    return std::sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
}

Position detectarParede(const std::vector<float>& pose, float distancia_sonar, bool esquerda) {
    float angulo = pose[2];

    float paredeX = pose[0];
    float paredeY = pose[1];

    float sinAng = sin(angulo);
    float cosAng = cos(angulo);

    if (esquerda){
        paredeX -=  distancia_sonar * sinAng;
        paredeY +=  distancia_sonar * cosAng;
    } else{
        paredeX +=  distancia_sonar * sinAng;
        paredeY -=  distancia_sonar * cosAng;        
    }

    Position pontoParede = {paredeX, paredeY};
    return {pontoParede};  
}

bool estaTrancado(const std::vector<Position>& posicoes, int ultimos = 10) {
    if (posicoes.size() < ultimos) return false;

    const Position& ref = posicoes[posicoes.size() - ultimos];
    for (size_t i = posicoes.size() - ultimos + 1; i < posicoes.size(); ++i) {
        if (!posicoes[i].isEqual(ref)) {
            return false;
        }
    }
    return true;
}

void Action::followTheWalls(std::vector<float> lasers, std::vector<float> sonars, std::vector<float> pose)
{
    auto [minPos, minDist] = findMinPosition(sonars);
    roboPosicao = {pose[0], pose[1], pose[2]};
    sonares = sonars;

    positionArray.push_back(roboPosicao);

    if (!firstInfo){
        firstMinDistPos = minPos;
        // verifica qual lado está mais próximo de uma parede
        if (firstMinDistPos >= 12 || firstMinDistPos <= 3){ 
            side = LEFT;
        }else{
            side = RIGHT;
        }
        firstInfo = true;
    }

    if (side == LEFT){
        // Parede mais proxima a frente pela esquerda
        if (minPos>0 && minPos<5){
            linVel=0.0; angVel=-0.2; // gira H
        }else 
            // Parede mais proxima atrás pela esquerda
            if (minPos>10){
                linVel=0.0; angVel=0.2; // gira AH
            }
        // Apenas quando a menor posição for 0 >> Alinhamento com a parede
        else{
            linVel=0.25; angVel=0.0; // anda reto

            if(sonars[0]<0.7){
                angVel=-0.2; // gira H
            }else
                if(sonars[0]>0.9){
                    angVel=0.2; // gira AH
                }
            //if (sonars[0]!=0.8){
            //    angVel = 2*(sonars[0]-0.8);
            //}
        }

        std::cout << sonars[0] << std::endl;
        // Anti colisão frontal
        if (sonars[3] <= 1.1 || sonars[4] <= 1.1){
            linVel= 0.0; angVel=-0.5; // rotação em sentido horário
        }
    }


    if (estaTrancado(positionArray)) {
        std::cout << "O robô está trancado!\n";
        linVel= 0.1;
    }
}

void Action::testMode(std::vector<float> lasers, std::vector<float> sonars, std::vector<float> pose)
{
    roboPosicao = {pose[0], pose[1], pose[2]};
    sonares = sonars;
    positionArray.push_back(roboPosicao);

    PID pid = { 0.1f, 0.2f, 0.02f }; // parâmetros do PID

    float x = pose[0] * scaleFactor;
    float y = pose[1] * scaleFactor;
    float yawAtual = pose[2];
    float yawIdeal = yawGradiente(campoPotencial, x, y);

    Controle ctrl = controleRobo(yawAtual, yawIdeal, pid);
    linVel = ctrl.linVel;
    angVel = ctrl.angVel;
}

void Action::manualRobotMotion(MovingDirection direction, std::vector<float> sonars, std::vector<float> pose)
{
    roboPosicao = {pose[0], pose[1], pose[2]};
    sonares = sonars;
    positionArray.push_back(roboPosicao);

    if(direction == FRONT){
        linVel= 0.5; angVel= 0.0;
    }else if(direction == BACK){
        linVel=-0.5; angVel= 0.0;
    }else if(direction == LEFT){
        linVel= 0.0; angVel= 0.5;
    }else if(direction == RIGHT){
        linVel= 0.0; angVel=-0.5;
    }else if(direction == STOP){
        linVel= 0.0; angVel= 0.0;
    }
    
    if (sonars[3] <= 1.1 || sonars[4] <= 1.1){
        linVel= 0.0; 
    }
}

void Action::correctVelocitiesIfInvalid()
{
    float b=0.38;

    float leftVel  = linVel - angVel*b/(2.0);
    float rightVel = linVel + angVel*b/(2.0);

    float VELMAX = 0.5;

    float absLeft = fabs(leftVel);
    float absRight = fabs(rightVel);

    if(absLeft>absRight){
        if(absLeft > VELMAX){
            leftVel *= VELMAX/absLeft;
            rightVel *= VELMAX/absLeft;
        }
    }else{
        if(absRight > VELMAX){
            leftVel *= VELMAX/absRight;
            rightVel *= VELMAX/absRight;
        }
    }
    
    linVel = (leftVel + rightVel)/2.0;
    angVel = (rightVel - leftVel)/b;
}

float Action::getLinearVelocity()
{
    return linVel;
}

float Action::getAngularVelocity()
{
    return angVel;
}

bool salvouMapa = false;
bool carregouMapa = false;
MotionControl Action::handlePressedKey(char key)
{
    MotionControl mc;

    if(key=='1'){
        mc.mode=MANUAL;
        mc.direction=STOP;
    }else if(key=='2'){
        mc.mode=WANDER;
        mc.direction=AUTO;
    }else if(key=='3'){
        mc.mode=FARFROMWALLS;
        mc.direction=AUTO;
    }else if(key=='4'){
        mc.mode=FOLLOWWALLS;
        mc.direction=AUTO;
    }else if(key=='5'){
        mc.mode=TESTMODE;
        mc.direction=AUTO;
    }else if(key=='w' or key=='W'){
        mc.mode=MANUAL;
        mc.direction = FRONT;
    }else if(key=='s' or key=='S'){
        mc.mode=MANUAL;
        mc.direction = BACK;
    }else if(key=='a' or key=='A'){
        mc.mode=MANUAL;
        mc.direction = LEFT;
    }else if(key=='d' or key=='D'){
        mc.mode=MANUAL;
        mc.direction = RIGHT;
    }else if(key==' '){
        mc.mode=MANUAL;
        mc.direction = STOP;
    }else if(key=='m' or key=='M'){  // Salva mapa 
        if(!salvouMapa){
            salvaMatriz(matrizMundo, "matriz.txt");
            salvouMapa = true;
        }else{
            salvouMapa = false;
        }
    }else if(key=='l' or key=='L'){  // Carrega mapa 
        if(!carregouMapa){
            matrizMundo = loadMatrix("matriz.txt");
            carregouMapa = true;
        }else{
            carregouMapa = false;
        }
    }
    
    return mc;
}

