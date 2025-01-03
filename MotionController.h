#ifndef MOTIONCONTROLLER_H
#define MOTIONCONTROLLER_H

using namespace std;
#include <vector>
#include <memory>
#include <iostream>

#include "CommandInfo.h"
#include "TrajectoryProfile.h"
#include "ScurveProfile.h"
#include "TrapezoidalProfile.h"

class MotionController
{
public:
    MotionController(int degreesOfFreedom);
    ~MotionController();
    void setScurveProfile(shared_ptr<ScurveProfile> prof);
    void setTrapezoidalProfile(shared_ptr<TrapezoidalProfile> prof);
    void setCurrentPose(vector<double> curPose);
    void setVelocityProfParam(vector<double> targetPose, vector<double> targetVels, vector<double> targetAccs, vector<double> targetJerks);
    vector<double> getCmdPose(void);
    vector<double> getCmdVels(void);
    vector<double> getCmdAccs(void);
    bool makeLinearProf(shared_ptr<TrajectoryProfile> prof, CommandInfo* cmdInfo, vector<double> startPose);
    bool isEnableToExecImposedProf();
    bool execCmd(double cycleTime);

private:
    int dof;
    int execProfId;
    int imposedProfId;
    int storedProfNum;
    
    vector<double> curPose;
    vector<double> curVels;
    vector<double> prePose;
    vector<double> preVels;
    vector<double> curAccs;
    vector<CommandInfo> cmdContainer;
    vector<shared_ptr<TrajectoryProfile>> profContainer;

};

#endif //MOTIONCONTROLLER_H