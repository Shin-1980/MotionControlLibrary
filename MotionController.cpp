#include "MotionController.h"


MotionController::MotionController(int degreesOfFreedom):
    dof(degreesOfFreedom),        
    execProfId(-1),
    imposedProfId(-1),
    maxProfNum(8),
    storedProfNum(0),
    curPose(dof, 0.0f),
    curVels(dof, 0.0f),
    prePose(dof, 0.0f),
    preVels(dof, 0.0f),
    curAccs(dof, 0.0f)
{
}

void MotionController::setScurveProfile(ScurveProfile* &prof) 
{
    this->profContainer.push_back(prof);
}

void MotionController::setTrapezoidalProfile(TrapezoidalProfile* &prof) 
{
    this->profContainer.push_back(prof);
}

void MotionController::setCurrentPose(std::vector<double> curPose)
{
    this->curPose = curPose;
    this->prePose = curPose;
}

void MotionController::setCmd(std::vector<double> targetPose, std::vector<double> targetVels, std::vector<double> targetAccs)
{
    CommandInfo cmd;
    this->cmdContainer.push_back(cmd);

    if (this->storedProfNum > this->maxProfNum) {
        //std::cout << "The number of profiler exceeds the limit" << std::endl;
    }

    this->cmdContainer[this->storedProfNum].setParam(targetPose, targetVels, targetAccs);
    this->storedProfNum++;
}

std::vector<double> MotionController::getCmdPose(void){
    return this->curPose;
}

std::vector<double> MotionController::getCmdVels(void){
    return this->curVels;
}

std::vector<double> MotionController::getCmdAccs(void){
    return this->curAccs;
}

bool MotionController::makeLinearProf(TrajectoryProfile* &prof, CommandInfo* cmdInfo, std::vector<double> startPose) {
    
    cmdInfo->setStartPose(startPose);

    std::vector<double> times(this->dof, 0.0f);
    std::vector<double> unsignedTotalDis = cmdInfo->getUnsignedTotalDistance();
    std::vector<double> targetVel = cmdInfo->getTargetVels();

    for(size_t i=0; i<this->dof; i++) {
        times[i] = unsignedTotalDis[i] / targetVel[i];
    }
    
    double maxTime = 0.0;

    for (size_t i=0;i<this->dof;i++){
        if (maxTime < times[i]){
            maxTime = times[i];
            cmdInfo->setBaseCoordinate(i);
        }
    }

    std::vector<double> accs = cmdInfo->getTargetAccs();
    std::vector<double> vels = cmdInfo->getTargetVels();
    std::vector<double> diss = cmdInfo->getUnsignedTotalDistance();

    for (size_t i=0;i<this->dof;i++){
        if (accs[i] < 0 || vels[i] < 0)
            return false;
    }

    double acc = accs[cmdInfo->getBaseCoordinate()];
    double vel = vels[cmdInfo->getBaseCoordinate()];
    double dis = diss[cmdInfo->getBaseCoordinate()];

    return prof->makeProf(acc, vel, dis);
}

bool MotionController::isEnableToExecImposedProf() {
    if (this->imposedProfId != -1 && (this->profContainer[this->execProfId]->isDecelerating() || this->profContainer[this->execProfId]->isDone())){
        if (this->profContainer[this->execProfId]->decTime < this->profContainer[this->imposedProfId]->totalTime)
            return true;
        else
            if (this->profContainer[this->execProfId]->elapsedTime > (this->profContainer[this->execProfId]->totalTime - this->profContainer[this->imposedProfId]->accTime)) return true;
    }

    return false;
}

bool MotionController::execCmd(double cycleTime) {

    if (this->execProfId == -1){
        if (this->storedProfNum == 0) {
            return false;
        }
        else {
            this->execProfId = 0;
            if (!this->makeLinearProf(this->profContainer[this->execProfId],
                                        &this->cmdContainer[this->execProfId], this->curPose)) 
                return false;
        }
    }
    
    if (this->imposedProfId == -1 && this->execProfId < this->storedProfNum - 1) {
        this->imposedProfId = this->execProfId + 1;
        
        if (!this->makeLinearProf(this->profContainer[this->imposedProfId],
                                    &this->cmdContainer[this->imposedProfId], 
                                    this->cmdContainer[this->execProfId].getTargetPose()))
            return false;
    }

    bool isExec = this->profContainer[execProfId]->calDis(cycleTime);        

    if (isExec){
        double curDis = this->profContainer[this->execProfId]->getCurDis();
        int baseCoordinate = this->cmdContainer[this->execProfId].getBaseCoordinate();
        std::vector<double> totalDis = this->cmdContainer[this->execProfId].getUnsignedTotalDistance();

        double rate = curDis / totalDis[baseCoordinate];
        
        std::vector<double> startPose = this->cmdContainer[this->execProfId].getStartPose();
        std::vector<double> signedTotalDis = this->cmdContainer[this->execProfId].getSignedTotalDistance();

        for (size_t i=0;i<this->dof;i++){
            this->curPose[i] = startPose[i] + signedTotalDis[i] * rate;
        }
    }
    else{
        this->curPose = this->cmdContainer[this->execProfId].getTargetPose();
    }

    if (this->isEnableToExecImposedProf()) {
        if (this->profContainer[this->imposedProfId]->calDis(cycleTime)){
            double ipDis = this->profContainer[imposedProfId]->getCurDis();
            double ipBaseCoordinate = this->cmdContainer[this->imposedProfId].getBaseCoordinate();
            std::vector<double> ipTotalDis = this->cmdContainer[this->imposedProfId].getUnsignedTotalDistance();
            double ipRate = ipDis / ipTotalDis[ipBaseCoordinate];  

            std::vector<double> signedTotalDis = this->cmdContainer[this->imposedProfId].getSignedTotalDistance();
            for (size_t i=0;i<this->dof;i++){
                this->curPose[i] += signedTotalDis[i] * ipRate;
            }
        }
    }

    for (size_t i=0;i<this->dof;i++) {
        this->curVels[i] = (this->curPose[i] - this->prePose[i]) / cycleTime;
        this->curAccs[i] = (this->curVels[i] - this->preVels[i]) / cycleTime;

        this->prePose[i] = this->curPose[i];
        this->preVels[i] = this->curVels[i];
    }

    if (this->profContainer[this->execProfId]->isDone()){
        if (this->imposedProfId == -1) {
            std::vector<double> endPos = this->cmdContainer[this->execProfId].getTargetPose();
            for (size_t i=0;i<this->dof;i++) {
                this->curVels[i] = 0.0;
                this->preVels[i] = 0.0;
                this->curPose[i] = endPos[i];
                this->execProfId = -1;
            }
            return false;
        }
        else {
            this->execProfId = this->imposedProfId;
            this->imposedProfId = -1;
        }
    }
    return true;

}
