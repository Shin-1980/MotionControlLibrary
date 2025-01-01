#include "CommandInfo.h"

CommandInfo::CommandInfo() :
    dof(6),
    baseCoordinate(0),
    targetPose(dof, 0.0f),
    targetVels(dof, 0.0f),
    targetAccs(dof, 0.0f),
    targetJerks(dof, 0.0f),
    startPose(dof, 0.0f)
{
}

CommandInfo::~CommandInfo()
{
}

void CommandInfo::setVelocityProfParam(vector<double> targetPose, vector<double> targetVels, vector<double> targetAccs, vector<double> targetJerks)
{
    this->targetPose = targetPose;
    this->targetVels = targetVels;
    this->targetAccs = targetAccs;
    this->targetJerks = targetJerks;
}

void CommandInfo::setStartPose(vector<double> startPose)
{
    this->startPose = startPose;
}

void CommandInfo::setBaseCoordinate(int base)
{
    this->baseCoordinate = base;
}

vector<double> CommandInfo::getTargetPose(void) 
{
    return this->targetPose;
}

vector<double> CommandInfo::getTargetVels(void) 
{
    return this->targetVels;
}

vector<double> CommandInfo::getTargetAccs(void) 
{
    return this->targetAccs;    
}

vector<double> CommandInfo::getTargetJerks(void) 
{
    return this->targetJerks;    
}

vector<double> CommandInfo::getStartPose(void) 
{
    return this->startPose;
}

int CommandInfo::getBaseCoordinate(void) 
{
    return this->baseCoordinate;
}

vector<double> CommandInfo::getUnsignedTotalDistance(void) 
{
    vector<double> unsignedTotalDis(this->dof);
    for (size_t i = 0; i < this->dof; ++i) {
        unsignedTotalDis[i] = fabs(this->targetPose[i] - this->startPose[i]);
    }
    return unsignedTotalDis;
}

vector<double> CommandInfo::getSignedTotalDistance(void) 
{
    vector<double> signedTotalDis(this->dof);
    for (size_t i = 0; i < this->dof; ++i) {
        signedTotalDis[i] = this->targetPose[i] - this->startPose[i];
    }
    return signedTotalDis;
}
