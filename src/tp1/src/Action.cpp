#include "Action.h"
#include "Utils.h"
#include "Mapping.hpp"

#include <vector>

// Variables

Position robotPosition = {0.0f, 0.0f, 0.0f};
std::vector<float> lasers;

// Methods

Action::Action()
{
    linVel = 0.0;
    angVel = 0.0;
}

void Action::exploreEnvironment(
    std::vector<float> lasersData, 
    std::vector<float> sonarsData, 
    std::vector<float> poseData
){
    // Sensing: Get robot position and laser data from /pose and /lasers topics
    robotPosition = {poseData[0], poseData[1], poseData[2]};
    lasers = lasersData;

}

void Action::manualRobotMotion(
    MovingDirection direction,
    std::vector<float> lasersData, 
    std::vector<float> sonarsData, 
    std::vector<float> poseData
){
    // Sensing: Get robot position and laser data from /pose and /lasers topics
    robotPosition = {poseData[0], poseData[1], poseData[2]};
    lasers = lasersData;

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
        mc.mode=EXPLORE;
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

