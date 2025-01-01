#ifndef MOTIONCONTROLLER_H
#define MOTIONCONTROLLER_H

using namespace std;
#include <vector>

#include "CommandInfo.h"
#include "TrajectoryProfile.h"
#include "ScurveProfile.h"
#include "TrapezoidalProfile.h"

class MotionController
{
public:
    int dof;
    int execProfId;
    int imposedProfId;
    int maxProfNum;
    int storedProfNum;
    
    vector<double> curPose;
    vector<double> curVels;
    vector<double> prePose;
    vector<double> preVels;
    vector<double> curAccs;
    vector<CommandInfo> cmdContainer;
    vector<TrajectoryProfile*> profContainer;

    MotionController(int degreesOfFreedom);
    void setScurveProfile(ScurveProfile* &prof);
    void setTrapezoidalProfile(TrapezoidalProfile* &prof);
    void setCurrentPose(vector<double> curPose);
    void setCmd(vector<double> targetPose, vector<double> targetVels, vector<double> targetAccs);
    vector<double> getCmdPose(void);
    vector<double> getCmdVels(void);
    vector<double> getCmdAccs(void);
    bool makeLinearProf(TrajectoryProfile* &prof, CommandInfo* cmdInfo, vector<double> startPose);
    bool isEnableToExecImposedProf();
    bool execCmd(double cycleTime);

};

#endif //MOTIONCONTROLLER_H