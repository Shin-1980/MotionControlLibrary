#ifndef SCURVEPROFILE_H
#define SCURVEPROFILE_H

using namespace std;
#include <cmath>
#include "TrajectoryProfile.h"
#include "CommandInfo.h"

class ScurveProfile : public TrajectoryProfile
{
public:
    ScurveProfile();
    ~ScurveProfile();
    double getCurDis(void);
    bool isDone(void);
    bool isDecelerating(void);
    bool makeVelProf(double dis, double vel, double acc, double jerk);
    bool calDis(double cycleTime);

private:
    bool isCreated;

    double  initTime[7];
	double  diffTime[7];
	double	initVel[7];
	double  diffVel[7];
	double  initPos[7];
	double  diffPos[7];

	double	curSecTime;
	double	distance;
	double	targetVel;
	int areaNo;
    double accDec;
    double jerk;
    double curDis;

    void fillAllProfile(); 
    void makeProfileInAccDecPhase();

};

#endif //SCURVEPROFILE_H