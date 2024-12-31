#include "CommandInfo.h"

CommandInfo::CommandInfo() :
    dof(6),
    baseCoordinate(0),
    targetPose(dof, 0.0f),
    targetVels(dof, 0.0f),
    targetAccs(dof, 0.0f),
    startPose(dof, 0.0f)
{
    // initialization
}

void CommandInfo::setParam(std::vector<double> targetPose, std::vector<double> targetVels, std::vector<double> targetAccs)
{
    this->targetPose = targetPose;
    this->targetVels = targetVels;
    this->targetAccs = targetAccs;
}

void CommandInfo::setStartPose(std::vector<double> startPose)
{
    this->startPose = startPose;
}

void CommandInfo::setBaseCoordinate(int base)
{
    this->baseCoordinate = base;
}

std::vector<double> CommandInfo::getTargetPose(void) 
{
    return this->targetPose;
}

std::vector<double> CommandInfo::getTargetVels(void) 
{
    return this->targetVels;
}

std::vector<double> CommandInfo::getTargetAccs(void) 
{
    return this->targetAccs;    
}

std::vector<double> CommandInfo::getStartPose(void) 
{
    return this->startPose;
}

int CommandInfo::getBaseCoordinate(void) 
{
    return this->baseCoordinate;
}

std::vector<double> CommandInfo::getUnsignedTotalDistance(void) 
{
    std::vector<double> unsignedTotalDis(this->dof);
    for (size_t i = 0; i < this->dof; ++i) {
        unsignedTotalDis[i] = std::fabs(this->targetPose[i] - this->startPose[i]);
    }
    return unsignedTotalDis;
}

std::vector<double> CommandInfo::getSignedTotalDistance(void) 
{
    std::vector<double> signedTotalDis(this->dof);
    for (size_t i = 0; i < this->dof; ++i) {
        signedTotalDis[i] = this->targetPose[i] - this->startPose[i];
    }
    return signedTotalDis;
}
