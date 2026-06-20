#ifndef ACTION_H
#define ACTION_H

#include <vector>

enum MotionMode {MANUAL, EXPLORE};
enum MovingDirection {STOP, FRONT, BACK, LEFT, RIGHT, AUTO};

typedef struct
{
    MotionMode mode;
    MovingDirection direction;
} MotionControl;

class Action
{
public:
    Action();
    
    void manualRobotMotion(
        MovingDirection direction, 
        std::vector<float> lasers, 
        std::vector<float> sonars, 
        std::vector<float> pose
    );
    void exploreEnvironment(
        std::vector<float> lasers, 
        std::vector<float> sonars, 
        std::vector<float> pose
    );

    MotionControl handlePressedKey(char key);

    void correctVelocitiesIfInvalid();
    float getLinearVelocity();
    float getAngularVelocity();

private:
    float linVel;
    float angVel;
    float yawIntegral;
    float yawPreviousError;
};

#endif // ACTION_H
