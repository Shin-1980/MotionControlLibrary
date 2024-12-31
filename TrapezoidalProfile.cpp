#include "TrapezoidalProfile.h"

TrapezoidalProfile::TrapezoidalProfile():
    isCreated(false),
    constTime(0.0f),
    accConstTime(0.0f),
    acc(0.0f),
    startVel(0.0f),
    targetVel(0.0f),
    endVel(0.0f),
    accDis(0.0f),
    constDis(0.0f),
    decDis(0.0f),
    totalDis(0.0f),
    curDis(0.0f),
    cmdInfo()
{    
}

double TrapezoidalProfile::getCurDis(void)
{
    return this->curDis;
}

bool TrapezoidalProfile::isDone(void)
{
    if (this->elapsedTime >= this->totalTime) 
        return true;
    else
        return false;
}

bool TrapezoidalProfile::isDecelerating(void) 
{
    if ((this->accTime + this->constTime) < this->elapsedTime && this->elapsedTime < this->totalTime) 
        return true;
    else
        return false;
}

bool TrapezoidalProfile::makeProf(double acc, double vel, double dis) 
{
    if (acc <= 0 || vel <= 0 || dis <= 0)
        return false;

    this->acc = std::abs(acc);
    this->targetVel = std::abs(vel);

    this->totalDis = abs(dis); 
    this->elapsedTime = 0;
    this->accTime = this->targetVel / this->acc;
    this->decTime = this->accTime;        
    this->accDis = 0.5 * this->targetVel * this->accTime;
    this->decDis = 0.5 * this->targetVel * this->decTime;  
    this->constDis = this->totalDis - (this->accDis + this->decDis);
    
    if (this->constDis < 0) {
        double tmp = 2. * (this->acc * this->acc) * this->totalDis / (this->acc + this->acc);
        if (tmp > 0)
            this->targetVel = sqrt(tmp);
        else
            return false;
        this->accTime = this->targetVel / this->acc;
        this->decTime = this->accTime;        
        this->accDis = 0.5 * this->targetVel * this->accTime;
        this->decDis = 0.5 * this->targetVel * this->decTime; 
        this->constDis = 0;
    }
        
    this->constTime = this->constDis / this->targetVel;
    this->accConstTime = this->accTime + this->constTime;
    this->totalTime = this->accConstTime + this->decTime;

    this->isCreated = true;

    return this->isCreated; 
}

bool TrapezoidalProfile::calDis(double cycleTime) 
{
    if (!this->isCreated)
        return false;

    this->elapsedTime += cycleTime;
    this->curDis = 0;

    if (this->elapsedTime < this->accTime)
        this->curDis = 0.5 * this->elapsedTime * this->acc * this->elapsedTime;

    else if (this->elapsedTime < this->accConstTime)
        this->curDis = this->accDis + this->targetVel * (this->elapsedTime - this->accTime);
    else {
        double complementaryTime = this->totalTime - this->elapsedTime;
        this->curDis = this->totalDis - 0.5 * complementaryTime * this->acc * complementaryTime;
    }

    if (this->totalTime < this->elapsedTime) {
        this->curDis = this->totalDis;
        return false;
    }

    return true;
}
