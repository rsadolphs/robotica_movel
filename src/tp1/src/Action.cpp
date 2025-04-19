#include "Action.h"
#include "Utils.h"

#include <vector>
#include <iostream>
#include <limits>

/*
std::vector<float> smoothVector(const std::vector<float>& inputVector, int N) {
    // O tamanho do vetor resultante será o tamanho do vetor original dividido por N
    int resultSize = inputVector.size() / N;
    std::vector<float> smoothedVector(resultSize, 0.0f);  // Vetor resultante da suavização

    // Iterando em blocos de N em N
    for (int i = 0; i < resultSize; ++i) {
        float sum = 0.0f;
        int count = 0;

        // Somando os valores dentro de cada bloco de tamanho N
        for (int j = i * N; j < (i + 1) * N && j < inputVector.size(); ++j) {
            sum += inputVector[j];
            count++;
        }

        // Calculando a média e armazenando no vetor de resultado
        smoothedVector[i] = sum / count;
    }

    return smoothedVector;
}
*/

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

Action::Action()
{
    linVel = 0.0;
    angVel = 0.0;
}

void Action::avoidObstacles(std::vector<float> lasers, std::vector<float> sonars)
{
    // TODO: COMPLETAR FUNCAO
    // const int blockSize = 5;
    // std::vector<float> smoothLasers = smoothVector(lasers, blockSize);
    float minDistance = 0.0f;
    int minDistPos = 0;

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

}

void Action::manualRobotMotion(MovingDirection direction)
{
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

MotionControl Action::handlePressedKey(char key)
{
    MotionControl mc;
    mc.mode=MANUAL;
    mc.direction=STOP;

    if(key=='1'){
        mc.mode=MANUAL;
        mc.direction=STOP;
    }else if(key=='2'){
        mc.mode=WANDER;
        mc.direction=AUTO;
    }else if(key=='3'){
        mc.mode=FARFROMWALLS;
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
    }
    
    return mc;
}

