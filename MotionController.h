#ifndef MOTIONCONTROLLER_H
#define MOTIONCONTROLLER_H

//using namespace std;
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
    
    std::vector<double> curPose;
    std::vector<double> curVels;
    std::vector<double> prePose;
    std::vector<double> preVels;
    std::vector<double> curAccs;
    std::vector<CommandInfo> cmdContainer;
    std::vector<TrajectoryProfile*> profContainer;

    MotionController(int degreesOfFreedom);
    void setScurveProfile(ScurveProfile* &prof);
    void setTrapezoidalProfile(TrapezoidalProfile* &prof);
    void setCurrentPose(std::vector<double> curPose);
    void setCmd(std::vector<double> targetPose, std::vector<double> targetVels, std::vector<double> targetAccs);
    std::vector<double> getCmdPose(void);
    std::vector<double> getCmdVels(void);
    std::vector<double> getCmdAccs(void);
    bool makeLinearProf(TrajectoryProfile* &prof, CommandInfo* cmdInfo, std::vector<double> startPose);
    bool isEnableToExecImposedProf();
    bool execCmd(double cycleTime);

};

#endif //MOTIONCONTROLLER_H