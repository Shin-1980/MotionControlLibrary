#ifndef COMMANDINFO_H
#define COMMANDINFO_H

using namespace std;
#include <vector>
#include <cmath>

class CommandInfo{
public:
    CommandInfo();
    
    void setVelocityProfParam(vector<double> targetPose, vector<double> targetVels, vector<double> targetAccs, vector<double> targetJerks);
    void setStartPose(std::vector<double> startPose);
    void setBaseCoordinate(int base);
    std::vector<double> getTargetPose(void);
    std::vector<double> getTargetVels(void);
    std::vector<double> getTargetAccs(void);
    std::vector<double> getTargetJerks(void);
    std::vector<double> getStartPose(void);
    int getBaseCoordinate(void);
    std::vector<double> getUnsignedTotalDistance(void);
    std::vector<double> getSignedTotalDistance(void);

private:
   int dof;
    int baseCoordinate;
    std::vector<double> targetPose;
    std::vector<double> targetVels;
    std::vector<double> targetAccs;
    std::vector<double> targetJerks;
    std::vector<double> startPose;

};

#endif //COMMANDINFO_H