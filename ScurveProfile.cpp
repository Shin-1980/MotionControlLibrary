#include "ScurveProfile.h"

ScurveProfile::ScurveProfile():
    isCreated(false),
    curSecTime(0.0f),
    distance(0.0f),
    targetVel(0.0f),
    areaNo(0),
    accDec(0.0f),
    jerk(0.0f),
    curDis(0.0f)
{
}

double ScurveProfile::getCurDis(void){
    return this->curDis;
}

bool ScurveProfile::isDone(void){

    if (this->curSecTime >= this->initTime[6] + this->diffTime[6]) 
        return true;
    else
        return false;
}

bool ScurveProfile::isDecelerating(void) {
    
    if ((this->initTime[3] + this->diffTime[3]) < this->curSecTime 
        && this->curSecTime < this->initTime[6] + this->diffTime[6]) 
        return true;
    
    return false;
}

bool ScurveProfile::makeVelProf(double dis, double vel, double acc, double jerk) {

    if (dis <= 0 || vel <= 0 || acc <= 0 || jerk <= 0)
        return false;

    this->jerk = jerk;
    this->accDec = acc; 
    this->targetVel = vel;
    this->distance = dis;

    this->diffTime[0] = this->accDec / this->jerk;
    this->diffVel[0] = this->accDec * (this->diffTime[0]) * 0.5;
    this->diffTime[2] = this->diffTime[0];
    this->diffVel[2] = this->diffVel[0];

    if (this->targetVel < fabs(this->diffVel[0] + this->diffVel[2] ))
    {
        this->diffTime[1] = 0.;
        this->diffVel[1] = 0.;
        this->diffPos[1] = 0.;
        this->accDec = sqrt(this->targetVel * this->jerk);
        
        this->diffTime[0] = fabs(this->accDec / this->jerk);
        this->diffVel[0] = this->accDec * (this->diffTime[0]) * 0.5;
        this->diffTime[2] = this->diffTime[0];
        this->diffVel[2] = this->diffVel[0];

        this->initVel[0] = 0.0;
        this->diffVel[0] = this->jerk * this->diffTime[0] * this->diffTime[0] * 0.5;
        this->initVel[1] = this->diffVel[0];
        this->initVel[2] = this->initVel[1];
        this->diffVel[2] = this->diffVel[0];
        this->diffPos[0] = this->jerk * this->diffTime[0] * this->diffTime[0] * this->diffTime[0] / 6.0;
        this->diffPos[2] = this->targetVel * this->diffTime[2] - this->jerk * this->diffTime[0] * this->diffTime[0] * this->diffTime[0] / 6.0;
    }
    else
    {
        this->diffVel[1] = this->targetVel - (this->diffVel[0] + this->diffVel[2]);
        this->diffTime[1] = fabs(this->diffVel[1] / this->accDec);
        this->initVel[0] = 0.0;
        this->initVel[1] = this->diffVel[0];
        this->initVel[2] = this->initVel[1] + this->diffVel[1];
        this->diffVel[2] = this->diffVel[0];
        this->jerk * this->diffTime[0] * this->diffTime[0] * this->diffTime[0] / 6.0;
        this->diffPos[1] = (this->initVel[1] + this->initVel[2]) * this->diffTime[1] * 0.5;        
        this->diffPos[0] = this->jerk * this->diffTime[0] * this->diffTime[0] * this->diffTime[0] / 6.0;
        this->diffPos[2]  = targetVel  * this->diffTime[2] - this->diffPos[0];
    }

    this->initVel[0] = 0.0;
    this->initVel[1] = this->diffVel[0];
    this->initVel[2] = this->initVel[1] + this->diffVel[1];

    this->fillAllProfile();

    if(this->distance < this->diffPos[0] + this->diffPos[1] 
                        + this->diffPos[2] + this->diffPos[4] 
                        + this->diffPos[5] + this->diffPos[6]) 
    {
        if(this->distance < 0.5 * this->accDec * this->accDec / this->jerk * this->accDec / this->jerk) {
            // correct both acceleration and target velotity
            this->accDec = cbrt(0.5 * this->distance * this->jerk * this->jerk); 
            this->targetVel = this->accDec * this->accDec / this->jerk;
        }
        else{
            // correct only target velocity
            this->targetVel = (sqrt((this->accDec / (this->jerk) * this->accDec / (this->jerk) + 4.0 * this->distance / this->accDec)) 
                            + this->accDec / (this->jerk) * (-1.)) * this->accDec * (0.5);
        }

        this->diffTime[0] = this->accDec / this->jerk;
        this->diffVel[0] = this->accDec * (this->diffTime[0]) * 0.5;
        this->diffTime[2] = this->diffTime[0];
        this->diffVel[2] = this->diffVel[0];

        this->diffVel[1] = this->targetVel - (this->diffVel[0] + this->diffVel[2]);
        this->diffTime[1] = fabs(this->diffVel[1] / this->accDec);
        this->initVel[0] = 0.0;
        this->initVel[1] = 0.0 + this->diffVel[0];
        this->initVel[2] = this->initVel[1] + this->diffVel[1];
        this->diffVel[2] = this->diffVel[0];

        this->diffPos[1] = (this->initVel[1] + this->initVel[2]) * this->diffTime[1] * 0.5;        
        this->diffPos[0] = 0.0 + this->jerk * this->diffTime[0] * this->diffTime[0] * this->diffTime[0] / 6.0;
        this->diffPos[2] = targetVel  * this->diffTime[2] - this->diffPos[0];

        this->fillAllProfile();
    }

    this->initVel[3] = this->targetVel;
    this->diffVel[3] = 0.0;
    this->diffTime[3] = (this->distance - (this->diffPos[0] + this->diffPos[1] 
                                        + this->diffPos[2] + this->diffPos[4] 
                                        + this->diffPos[5] + this->diffPos[6])) / this->targetVel;
    this->diffPos[3] = this->targetVel * this->diffTime[3];

    this->initTime[0] = 0.0;
    this->initPos[0] = 0.0;

    for (size_t i=1;i<7;i++) {
        this->initTime[i] = this->initTime[i-1] + this->diffTime[i-1];
        this->initPos[i] = this->initPos[i-1] + this->diffPos[i-1];
    }

    // temporal code
    this->accTime = this->diffTime[0] + this->diffTime[1] + this->diffTime[2];
    this->decTime = this->diffTime[4] + this->diffTime[5] + this->diffTime[6];
    this->totalTime = this->accTime + this->diffTime[3] + this->decTime;

    this->isCreated = true;

    return true;
}

bool ScurveProfile::calDis(double cycleTime) {

    if (!this->isCreated)
        return false;

    this->curSecTime += cycleTime;
    this->elapsedTime = this->curSecTime;

    while(this->areaNo < 7) 
    {
        if (this->curSecTime < this->initTime[this->areaNo] + this->diffTime[this->areaNo]){
            break;
        }
        else{
            this->areaNo++;
        }        
    }

    double timeInArea;
    double compTimeInArea;

    switch(this->areaNo) 
    {
    case 0:
        timeInArea = this->curSecTime;
        this->curDis = this->initPos[0]
                        + this->jerk * timeInArea * timeInArea * timeInArea / 6.0;
        break;
    case 1:
        timeInArea = this->curSecTime - this->initTime[this->areaNo];
        this->curDis = this->initPos[1]
                        + (this->initVel[1] + this->initVel[1] 
                        + this->accDec * timeInArea) * timeInArea * 0.5;
        break;
    case 2:
        timeInArea = this->curSecTime - this->initTime[this->areaNo];
        compTimeInArea = this->initTime[3] - this->curSecTime;
        this->curDis = this->initPos[2] + this->targetVel * timeInArea
                        - this->jerk * ((this->diffTime[2] * this->diffTime[2] * this->diffTime[2])
                        - (compTimeInArea * compTimeInArea * compTimeInArea)) / 6.0;
        break;
    case 3:
        timeInArea = this->curSecTime - this->initTime[this->areaNo];
        this->curDis = this->initPos[3] + this->targetVel * timeInArea;
        break;
    case 4:
        timeInArea = this->curSecTime - this->initTime[this->areaNo];
        this->curDis = this->initPos[4] + this->targetVel * timeInArea
                        - this->jerk * timeInArea * timeInArea * timeInArea / 6.0;
        break;
    case 5:
        timeInArea = this->curSecTime - this->initTime[this->areaNo];
        this->curDis = this->initPos[5] + (this->initVel[5] + this->initVel[5] - this->accDec * timeInArea) 
                        * timeInArea * 0.5;
        break;
    case 6:
        timeInArea = this->curSecTime - this->initTime[this->areaNo];
        compTimeInArea = this->initTime[6]+ this->diffTime[6] - this->curSecTime;
        this->curDis = this->initPos[6]
                        + this->jerk * ((this->diffTime[6] * this->diffTime[6] * this->diffTime[6])
                        - (compTimeInArea * compTimeInArea * compTimeInArea)) / 6.0;                                    
        break;
    case 7:
        this->curDis = this->distance;
        return false;
    }

    return true;
}

void ScurveProfile::fillAllProfile(){
        
    this->diffTime[6] = this->diffTime[0];
    this->diffVel[6] = this->diffVel[0] * -1.0;
    this->diffPos[6] = this->diffPos[0];
    this->diffTime[4] = this->diffTime[2];
    this->diffVel[4] = this->diffVel[2] * -1.0;
    this->diffPos[4] = this->diffPos[2];

    this->diffTime[5] = this->diffTime[1];
    this->diffVel[6] = this->diffVel[1] * -1.0;
    this->diffPos[5] = this->diffPos[1];	

    this->initVel[4] = this->targetVel;
    this->initVel[5] = this->initVel[4] + this->diffVel[4];
    this->initVel[6] = this->initVel[5] + this->diffVel[6];
}


