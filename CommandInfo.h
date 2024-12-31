#ifndef COMMANDINFO_H
#define COMMANDINFO_H

using namespace std;
#include <vector>
#include <cmath>

class CommandInfo{
public:
    int dof;
    int baseCoordinate;
    std::vector<double> targetPose;
    std::vector<double> targetVels;
    std::vector<double> targetAccs;
    std::vector<double> startPose;

    CommandInfo();
    void setParam(std::vector<double> targetPose, std::vector<double> targetVels, std::vector<double> targetAccs);
    void setStartPose(std::vector<double> startPose);
    void setBaseCoordinate(int base);
    std::vector<double> getTargetPose(void);
    std::vector<double> getTargetVels(void);
    std::vector<double> getTargetAccs(void);
    std::vector<double> getStartPose(void);
    int getBaseCoordinate(void);
    std::vector<double> getUnsignedTotalDistance(void);
    std::vector<double> getSignedTotalDistance(void);
};

#endif //COMMANDINFO_H