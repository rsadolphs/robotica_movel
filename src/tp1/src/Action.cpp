#include "Action.h"
#include "Potential.hpp"
#include "Utils.h"
#include "Mapping.hpp"

#include <vector>
#include <cmath>
#include <limits>

static float normalizeAngle(float angle)
{
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle <= -M_PI) angle += 2.0f * M_PI;
    return angle;
}

// Variables

Position robotPosition = {0.0f, 0.0f, 0.0f};
std::vector<float> lasers;

// Methods

Action::Action()
{
    linVel = 0.0;
    angVel = 0.0;
    yawIntegral = 0.0f;
    yawPreviousError = 0.0f;
}

void Action::exploreEnvironment(
    std::vector<float> lasersData, 
    std::vector<float> sonarsData, 
    std::vector<float> poseData
){
    (void)sonarsData;
    robotPosition = {poseData[0], poseData[1], poseData[2]};
    lasers = lasersData;

    std::vector<Cell> visitedCells = getVisitedCells();
    std::vector<Cell> frontiers = getFrontiers();

    if (!frontierSeen && !frontiers.empty())
    {
        frontierSeen = true;
    }

    if (frontiers.empty())
    {
        if (explorationStarted && frontierSeen)
        {
            explorationEnded = true;
            explorationStarted = false;
        }

        linVel = 0.0f;
        angVel = 0.0f;
        return;
    }

    Potential::updateRobotPose(robotPosition.x, robotPosition.y, robotPosition.theta);
    Potential::TargetYaw target = Potential::getLatestTargetYaw();

    if (!target.valid)
    {
        linVel = 0.0f;
        angVel = 0.0f;
        return;
    }

    float yawError = normalizeAngle(target.yaw - robotPosition.theta);
    const float dt = 0.05f;
    const float kp = 1.2f;
    const float ki = 0.02f;
    const float kd = 0.1f;

    yawIntegral += yawError * dt;
    float derivative = (yawError - yawPreviousError) / dt;
    yawPreviousError = yawError;

    float angular = kp * yawError + ki * yawIntegral + kd * derivative;
    const float maxAngular = 0.8f;
    if (angular > maxAngular) angular = maxAngular;
    if (angular < -maxAngular) angular = -maxAngular;

    float yawAbs = std::fabs(yawError);
    float linear = 0.0f;
    if (yawAbs < 0.25f) {
        linear = 0.18f;
    }
    else {
        linear = 0.0f;
    }

    linVel = linear;
    angVel = angular;
}

void Action::manualRobotMotion(
    MovingDirection direction,
    std::vector<float> lasersData, 
    std::vector<float> sonarsData, 
    std::vector<float> poseData
){
    (void)sonarsData;
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

bool Action::explorationEndedByNoFrontiers() const
{
    return explorationEnded;
}

void Action::clearExplorationEnded()
{
    explorationEnded = false;
}

MotionControl Action::handlePressedKey(char key)
{
    MotionControl mc;
    mc.mode=MANUAL;
    mc.direction=STOP;

    if (key != lastKey)
    {
        if (key == '2')
        {
            explorationStarted = true;
            explorationEnded = false;
            frontierSeen = false;
        }
        else if (key == '1')
        {
            explorationStarted = false;
            explorationEnded = false;
            frontierSeen = false;
        }
        else if (key == 'w' || key == 'W' || key == 's' || key == 'S' ||
                 key == 'a' || key == 'A' || key == 'd' || key == 'D' ||
                 key == ' ')
        {
            explorationStarted = false;
            explorationEnded = false;
            frontierSeen = false;
        }
        lastKey = key;
    }

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

