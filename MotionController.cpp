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

MotionController::~MotionController()
{
    profContainer.clear();
}

void MotionController::setScurveProfile(shared_ptr<ScurveProfile> prof) 
{
    this->profContainer.push_back(prof);
}

void MotionController::setTrapezoidalProfile(shared_ptr<TrapezoidalProfile> prof) 
{
    this->profContainer.push_back(prof);
}

void MotionController::setCurrentPose(vector<double> curPose)
{
    this->curPose = curPose;
    this->prePose = curPose;
}

void MotionController::setVelocityProfParam(vector<double> targetPose, vector<double> targetVels, vector<double> targetAccs, vector<double> targetJerks)
{
    CommandInfo cmd;
    this->cmdContainer.push_back(cmd);

    if (this->storedProfNum > this->maxProfNum) {
        //cout << "The number of profiler exceeds the limit" << endl;
    }

    this->cmdContainer[this->storedProfNum].setVelocityProfParam(targetPose, targetVels, targetAccs, targetJerks);
    this->storedProfNum++;
}


vector<double> MotionController::getCmdPose(void){
    return this->curPose;
}

vector<double> MotionController::getCmdVels(void){
    return this->curVels;
}

vector<double> MotionController::getCmdAccs(void){
    return this->curAccs;
}

bool MotionController::makeLinearProf(shared_ptr<TrajectoryProfile> prof, CommandInfo* cmdInfo, vector<double> startPose) {
    
    cmdInfo->setStartPose(startPose);

    vector<double> times(this->dof, 0.0f);
    vector<double> unsignedTotalDis = cmdInfo->getUnsignedTotalDistance();
    vector<double> targetVel = cmdInfo->getTargetVels();

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

    vector<double> accs = cmdInfo->getTargetAccs();
    vector<double> vels = cmdInfo->getTargetVels();
    vector<double> diss = cmdInfo->getUnsignedTotalDistance();
    vector<double> jerks = cmdInfo->getTargetJerks();

    for (size_t i=0;i<this->dof;i++){
        if (accs[i] < 0 || vels[i] < 0)
            return false;
    }

    double acc = accs[cmdInfo->getBaseCoordinate()];
    double vel = vels[cmdInfo->getBaseCoordinate()];
    double dis = diss[cmdInfo->getBaseCoordinate()];
    double jerk = jerks[cmdInfo->getBaseCoordinate()];

    for (int i=0;i<this->dof;i++){
        if (vel > vels[i]) vel = vels[i];
    }

    return prof->makeVelProf(dis, vel, acc, jerk);
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
        vector<double> totalDis = this->cmdContainer[this->execProfId].getUnsignedTotalDistance();

        double rate = curDis / totalDis[baseCoordinate];
        
        vector<double> startPose = this->cmdContainer[this->execProfId].getStartPose();
        vector<double> signedTotalDis = this->cmdContainer[this->execProfId].getSignedTotalDistance();

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
            vector<double> ipTotalDis = this->cmdContainer[this->imposedProfId].getUnsignedTotalDistance();
            double ipRate = ipDis / ipTotalDis[ipBaseCoordinate];  

            vector<double> signedTotalDis = this->cmdContainer[this->imposedProfId].getSignedTotalDistance();
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
            vector<double> endPos = this->cmdContainer[this->execProfId].getTargetPose();
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
