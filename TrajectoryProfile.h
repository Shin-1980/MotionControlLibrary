#ifndef TRAJECTORYPROFILE_H
#define TRAJECTORYPROFILE_H

using namespace std;

class TrajectoryProfile
{
public:
    double elapsedTime;
    double totalTime;
    double decTime;
    double accTime;

    virtual double getCurDis() = 0;
    virtual bool isDone() = 0;
    virtual bool isDecelerating() = 0;
    virtual bool makeProf(double acc, double vel, double dis) = 0;
    virtual bool calDis(double cycleTime) = 0;   

    TrajectoryProfile();

};

#endif //TRAJECTORYPROFILE_H