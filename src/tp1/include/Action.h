#ifndef ACTION_H
#define ACTION_H

#include <vector>

enum MotionMode {MANUAL, WANDER, FARFROMWALLS, FOLLOWWALLS};
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
    
    void manualRobotMotion(MovingDirection direction);
    void avoidObstacles(std::vector<float> lasers, std::vector<float> sonars);
    void keepAsFarthestAsPossibleFromWalls(std::vector<float> lasers, std::vector<float> sonars);
    void followTheWalls(std::vector<float> lasers, std::vector<float> sonars);

    MotionControl handlePressedKey(char key);

    void correctVelocitiesIfInvalid();
    float getLinearVelocity();
    float getAngularVelocity();

private:
    float linVel;
    float angVel;
};

#endif // ACTION_H
