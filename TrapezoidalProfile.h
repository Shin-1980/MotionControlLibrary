#ifndef TRAPEZOIDALPROFILE_H
#define TRAPEZOIDALPROFILE_H

using namespace std;
#include <cmath>

#include "TrajectoryProfile.h"
#include "CommandInfo.h"

class TrapezoidalProfile : public TrajectoryProfile
{
public:
    bool isCreated;
    double constTime;
    double accConstTime;

    double acc;    
    double startVel;
    double targetVel;
    double endVel;
    
    double accDis;
    double constDis;
    double decDis;
    double totalDis;
    double curDis;
    CommandInfo cmdInfo;

    TrapezoidalProfile();
    ~TrapezoidalProfile();
    double getCurDis(void);
    bool isDone(void);
    bool isDecelerating(void); 
    bool makeVelProf(double dis, double vel, double acc, double jerk); 
    bool calDis(double cycleTime); 
};

#endif //TRAPEZOIDALPROFILE_H