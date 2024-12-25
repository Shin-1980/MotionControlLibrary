#include <iostream>
#include <math.h>
#include <vector>
#include <queue>

#include <fstream>
#include <sstream>
#include <string>

class CommandInfo{
public:
    std::vector<double> targetPose;
    std::vector<double> targetVels;
    std::vector<double> targetAccs;
    std::vector<double> startPose;
    
    int baseCoordinate;
    int dof;

    CommandInfo() :
        dof(6),
        targetPose(dof, 0.0f),
        targetVels(dof, 0.0f),
        targetAccs(dof, 0.0f),
        startPose(dof, 0.0f),
        baseCoordinate(0)
    {
        // initialization
    }

    void setParam(std::vector<double> targetPose, std::vector<double> targetVels, std::vector<double> targetAccs){
        this->targetPose = targetPose;
        this->targetVels = targetVels;
        this->targetAccs = targetAccs;
    }

    void setStartPose(std::vector<double> startPose){
        this->startPose = startPose;
    }

    void setBaseCoordinate(int base){
        this->baseCoordinate = base;
    }

    std::vector<double> getTargetPose(void) {
        return this->targetPose;
    }

    std::vector<double> getTargetVels(void) {
        return this->targetVels;
    }

    std::vector<double> getTargetAccs(void) {
        return this->targetAccs;    
    }

    std::vector<double> getStartPose(void) {
        return this->startPose;
    }

    int getBaseCoordinate(void) {
        return this->baseCoordinate;
    }

    std::vector<double> getUnsignedTotalDistance(void) {
        std::vector<double> unsignedTotalDis(this->dof);
        for (size_t i = 0; i < this->dof; ++i) {
            unsignedTotalDis[i] = std::abs(this->targetPose[i] - this->startPose[i]);
        }
        return unsignedTotalDis;
    }

    std::vector<double> getSignedTotalDistance(void) {
        std::vector<double> signedTotalDis(this->dof);
        for (size_t i = 0; i < this->dof; ++i) {
            signedTotalDis[i] = this->targetPose[i] - this->startPose[i];
        }
        return signedTotalDis;
    }
};

class TrajectoryProfile{
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

    TrajectoryProfile():
        elapsedTime(0.0f),
        totalTime(0.0f),
        decTime(0.0f),
        accTime(0.0f)
    {
        // initialization
    }

};

class TrapezoidalProfile : public TrajectoryProfile{
public:
    bool isCreated;
    double constTime;
    double accConstTime;

    double acc;    
    double startVel;
    double targetVel;
    double endVel;
    
    double accDis;
    double constDis;
    double decDis;
    double totalDis;
    double curDis;
    CommandInfo cmdInfo;

    TrapezoidalProfile():
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
        // initilization
        
    }

    double getCurDis(void){
        return this->curDis;
    }

    bool isDone(void){
        if (this->elapsedTime >= this->totalTime) 
            return true;
        else
            return false;
    }

    bool isDecelerating(void) {
        if ((this->accTime + this->constTime) < this->elapsedTime && this->elapsedTime < this->totalTime) 
            return true;
        else
            return false;
    }

    bool makeProf(double acc, double vel, double dis) {

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

    bool calDis(double cycleTime) {

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
};

class ScurveProfile : public TrajectoryProfile{
public:
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
 
    ScurveProfile():
        isCreated(false),
        curSecTime(0.0f),
        distance(0.0f),
        targetVel(0.0f),
        areaNo(0),
        accDec(0.0f),
        jerk(0.0f),
        curDis(0.0f)
    {
        ;
    }

    double getCurDis(void){
        return this->curDis;
    }

    bool isDone(void){

        if (this->curSecTime >= this->initTime[6] + this->diffTime[6]) 
            return true;
        else
            return false;

    }

    bool isDecelerating(void) {
        
        if ((this->initTime[3] + this->diffTime[3]) < this->curSecTime 
          && this->curSecTime < this->initTime[6] + this->diffTime[6]) 
            return true;
        
        return false;
    }

    bool makeProf(double acc, double vel, double dis) {
    
        this->jerk = 1;
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

        return true;
    }

    bool calDis(double cycleTime) {

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

private:
    void fillAllProfile(){
         
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
};

class MotionController{
public:
    int dof;
    std::vector<double> curPose;
    std::vector<double> curVels;
    std::vector<double> prePose;
    std::vector<double> preVels;
    std::vector<double> curAccs;
    int execProfId;
    int imposedProfId;
    int maxProfNum;
    int storedProfNum;
    std::vector<CommandInfo> cmdContainer;
    std::vector<TrajectoryProfile*> profContainer;

    MotionController(int degreesOfFreedom):
        dof(degreesOfFreedom),        
        curPose(dof, 0.0f),
        curVels(dof, 0.0f),
        prePose(dof, 0.0f),
        preVels(dof, 0.0f),
        curAccs(dof, 0.0f),
        execProfId(-1),
        imposedProfId(-1),
        maxProfNum(8),
        storedProfNum(0)
    {
        // initialization
    }

    void setScurveProfile(ScurveProfile* &prof) {
        this->profContainer.push_back(prof);
    }

    void setTrapezoidalProfile(TrapezoidalProfile* &prof) {
        this->profContainer.push_back(prof);
    }

    void setCurrentPose(std::vector<double> curPose){
        this->curPose = curPose;
        this->prePose = curPose;
    }

    void setCmd(std::vector<double> targetPose, std::vector<double> targetVels, std::vector<double> targetAccs){
        CommandInfo cmd;
        this->cmdContainer.push_back(cmd);

        if (this->storedProfNum > this->maxProfNum) {
            std::cout << "The number of profiler exceeds the limit" << std::endl;
        }

        this->cmdContainer[this->storedProfNum].setParam(targetPose, targetVels, targetAccs);
        this->storedProfNum++;
    }

    std::vector<double> getCmdPose(void){
        return this->curPose;
    }

    std::vector<double> getCmdVels(void){
        return this->curVels;
    }

    std::vector<double> getCmdAccs(void){
        return this->curAccs;
    }

    bool makeLinearProf(TrajectoryProfile* &prof, CommandInfo* cmdInfo, std::vector<double> startPose) {
        
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

    bool isEnableToExecImposedProf() {
        if (this->imposedProfId != -1 && (this->profContainer[this->execProfId]->isDecelerating() || this->profContainer[this->execProfId]->isDone())){
            if (this->profContainer[this->execProfId]->decTime < this->profContainer[this->imposedProfId]->totalTime)
                return true;
            else
                if (this->profContainer[this->execProfId]->elapsedTime > (this->profContainer[this->execProfId]->totalTime - this->profContainer[this->imposedProfId]->accTime)) return true;
        }

        return false;
    }

    bool execCmd(double cycleTime) {

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

};

bool testCase01() {
    TrapezoidalProfile prof;

    double startPos = 0.0;
    double tarPos = 50;
    double acc = 100;
    double vel = 100;
    double cycleTime = 0.1;
    double prePos = startPos;
    double curPos = startPos;
    double preVel = 0;
    double curVel = 0;

    std::string filename = "./testCase/case1.csv";
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return false;
    }

    if (!prof.makeProf(acc,vel,tarPos - startPos)) return false;

    std::string line;
    while (std::getline(file, line) && prof.calDis(cycleTime)) { // Read the file line by line
        std::stringstream ss(line);   // Use stringstream to parse the line
        std::string value;
        std::vector<std::string> row; // Store the values of the current row

        while (std::getline(ss, value, ',')) { // Split by comma
            row.push_back(value);
        }

        double dis = prof.getCurDis();
        if (abs(dis - std::stof(row[0]) > 0.0001)) {
            return false;
        }
    }
    
    if (abs(prof.getCurDis() - tarPos > 0.0001)) {
        return false;
    }

    file.close();

    return true;
}

bool testCase02() {
    TrapezoidalProfile prof;

    double startPos = 0.0;
    double tarPos = 25;
    double acc = 100;
    double vel = 100;
    double cycleTime = 0.1;
    double prePos = startPos;
    double curPos = startPos;
    double preVel = 0;
    double curVel = 0;

    std::string filename = "./testCase/case2.csv";
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return 1;
    }

    if (!prof.makeProf(acc,vel,tarPos - startPos)) return false;

    std::string line;
    while (std::getline(file, line) && prof.calDis(cycleTime)) { // Read the file line by line
        std::stringstream ss(line);   // Use stringstream to parse the line
        std::string value;
        std::vector<std::string> row; // Store the values of the current row

        while (std::getline(ss, value, ',')) { // Split by comma
            row.push_back(value);
        }

        double dis = prof.getCurDis();
        
        if (abs(dis - std::stof(row[0]) > 0.0001)) {
            return false;
        }
    }
    
    if (abs(prof.getCurDis() - tarPos > 0.0001)) {
        return false;
    }

    file.close();

    return true;

}

bool testCase101(void){
    int dof = 6;
    MotionController mc(dof);

    TrapezoidalProfile* execPorf1 = new TrapezoidalProfile();

    mc.setTrapezoidalProfile(execPorf1);

    std::vector<double> curPose;
    curPose.push_back(1.5708f);
    curPose.push_back(-3.14159f);
    curPose.push_back(1.5708f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);

    mc.setCurrentPose(curPose);

    std::vector<double> targetPose;
    targetPose.push_back(0.693782f);
    targetPose.push_back(-2.36364f);
    targetPose.push_back(2.38406f);
    targetPose.push_back(2.1103f);
    targetPose.push_back(-1.07417f);
    targetPose.push_back(-1.14081f);

    std::vector<double> targetVelsRad;
    targetVelsRad.push_back(5.23598776f);
    targetVelsRad.push_back(5.23598776f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(10.47197551f);

    std::vector<double> targetAccsRad;
    targetAccsRad.push_back(17.45329252f);
    targetAccsRad.push_back(17.45329252f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(34.90658504f);
    mc.setCmd(targetPose, targetVelsRad, targetAccsRad);

    double cycleTime = 0.01;

    std::vector<double> cmdPose = curPose;

    std::string filename = "./testCase/case101.csv";
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return 1;
    }
    std::string line;
    while (std::getline(file, line) && mc.execCmd(cycleTime)) { // Read the file line by line
        std::stringstream ss(line);   // Use stringstream to parse the line
        std::string value;
        std::vector<std::string> row; // Store the values of the current row

        while (std::getline(ss, value, ',')) { // Split by comma
            row.push_back(value);
        }

        cmdPose = mc.getCmdPose();

        for (size_t i=0;i<dof;i++) {
            if (abs(cmdPose[i] - std::stof(row[i])) > 0.0001) return false;
        }
    }

    cmdPose = mc.getCmdPose();

    for (size_t i=0;i<dof;i++) {
        if (abs(cmdPose[i] - targetPose[i]) > 0.0001) return false;
    }

    return true;
}

bool testCase102(void){
        
    int dof = 6;
    MotionController mc(dof);

    TrapezoidalProfile* execPorf = new TrapezoidalProfile();
    mc.setTrapezoidalProfile(execPorf);

    std::vector<double> curPose;
    curPose.push_back(1.5708f);
    curPose.push_back(-3.14159f);
    curPose.push_back(1.5708f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);

    mc.setCurrentPose(curPose);

    std::vector<double> targetPose;
    targetPose.push_back(0.693782f);
    targetPose.push_back(-2.36364f);
    targetPose.push_back(2.38406f);
    targetPose.push_back(2.1103f);
    targetPose.push_back(-1.07417f);
    targetPose.push_back(-1.14081f);

    std::vector<double> targetVelsRad;

    targetVelsRad.push_back(0.52359878f);
    targetVelsRad.push_back(0.52359878f);
    targetVelsRad.push_back(0.65449847f);
    targetVelsRad.push_back(0.65449847f);
    targetVelsRad.push_back(0.65449847f);
    targetVelsRad.push_back(1.04719755f);

    std::vector<double> targetAccsRad;
    targetAccsRad.push_back(17.45329252f);
    targetAccsRad.push_back(17.45329252f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(34.90658504f);

    mc.setCmd(targetPose, targetVelsRad, targetAccsRad);

    double cycleTime = 0.01;

    std::vector<double> cmdPose = curPose;

    std::string filename = "./testCase/case102.csv";
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return 1;
    }

    std::string line;
    while (std::getline(file, line) && mc.execCmd(cycleTime)) { // Read the file line by line
        std::stringstream ss(line);   // Use stringstream to parse the line
        std::string value;
        std::vector<std::string> row; // Store the values of the current row

        while (std::getline(ss, value, ',')) { // Split by comma
            row.push_back(value);
        }

        cmdPose = mc.getCmdPose();

        for (size_t i=0;i<dof;i++) {
            if (abs(cmdPose[i] - std::stof(row[i])) > 0.0001) {
                return false;
            }
        }
    }

    if (mc.execCmd(cycleTime)) return false;

    cmdPose = mc.getCmdPose();
    for (size_t i=0;i<dof;i++) {
        if (abs(cmdPose[i] - targetPose[i]) > 0.0001) return false;
    }

    return true;
}

bool testCase109(){
    int dof = 6;
    MotionController mc(dof);

    TrapezoidalProfile* execPorf1 = new TrapezoidalProfile();
    TrapezoidalProfile* execPorf2 = new TrapezoidalProfile();
    TrapezoidalProfile* execPorf3 = new TrapezoidalProfile();

    mc.setTrapezoidalProfile(execPorf1);
    mc.setTrapezoidalProfile(execPorf2);
    mc.setTrapezoidalProfile(execPorf3);

    std::vector<double> curPose;
    curPose.push_back(1.5708f);
    curPose.push_back(-3.14159f);
    curPose.push_back(1.5708f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);
    mc.setCurrentPose(curPose);

    std::vector<double> targetPose;
    targetPose.push_back(0.693782f);
    targetPose.push_back(-2.36364f);
    targetPose.push_back(2.38406f);
    targetPose.push_back(2.1103f);
    targetPose.push_back(-1.07417f);
    targetPose.push_back(-1.14081f);

    std::vector<double> targetVelsRad;
    targetVelsRad.push_back(5.23598776f);
    targetVelsRad.push_back(5.23598776f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(10.47197551f);

    std::vector<double> targetAccsRad;
    targetAccsRad.push_back(17.45329252f);
    targetAccsRad.push_back(17.45329252f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(34.90658504f);
    mc.setCmd(targetPose, targetVelsRad, targetAccsRad);

    targetPose[0] = 0.456344;
    targetPose[1] = -2.54862;
    targetPose[2] = 2.16375;
    targetPose[3] = 1.78586;
    targetPose[4] = -0.350709;
    targetPose[5] = -1.29741;
    mc.setCmd(targetPose, targetVelsRad, targetAccsRad);

    targetPose[0] = -0.449;
    targetPose[1] = -0.890494;
    targetPose[2] = 2.60354;
    targetPose[3] = 2.06081;
    targetPose[4] = -0.0605401;
    targetPose[5] = -0.624552;
    mc.setCmd(targetPose, targetVelsRad, targetAccsRad);

    double cycleTime = 0.01;
    std::vector<double> cmdPose = curPose;
    std::string filename = "./testCase/case109.csv";

    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return 1;
    }

/*
    std::ofstream foutp;
    foutp.open(filename);
*/

    std::string line;
    while (std::getline(file, line) && mc.execCmd(cycleTime)) { // Read the file line by line
        std::stringstream ss(line);
        std::string value;
        std::vector<std::string> row;

        while (std::getline(ss, value, ',')) {
            row.push_back(value);
        }

//    while(mc.execCmd(cycleTime)) {
        //std::cout << "1097" << std::endl;
        cmdPose = mc.getCmdPose();

        for (size_t i=0;i<dof;i++) {
            if (abs(cmdPose[i] - std::stof(row[i])) > 0.0001) {
                return false;
            }
        }
    }

/*
        for (size_t i=0;i<dof;i++) {
            foutp << cmdPose[i] << ",";
        }    
        //std::cout << std::endl;
        foutp << std::endl;
    }
*/       
    if (mc.execCmd(cycleTime)) return false;

    cmdPose = mc.getCmdPose();
    for (size_t i=0;i<dof;i++) {
        if (abs(cmdPose[i] - targetPose[i]) > 0.0001) return false;
    }

    return true;

}

bool testCaseXXX(){
    int dof = 6;
    MotionController mc(dof);

    std::vector<double> curPose;
    curPose.push_back(1.5708f);
    curPose.push_back(-3.14159f);
    curPose.push_back(1.5708f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);

    mc.setCurrentPose(curPose);

    std::vector<double> targetPose;
    targetPose.push_back(1.68019f);
    targetPose.push_back(-2.35679f);
    targetPose.push_back(2.45324f);
    targetPose.push_back(2.56348f);
    targetPose.push_back(-0.904948f);
    targetPose.push_back(3.63343f);

    std::vector<double> targetVelsRad;
    targetVelsRad.push_back(5.23598776f);
    targetVelsRad.push_back(5.23598776f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(10.47197551f);

    std::vector<double> targetAccsRad;
    targetAccsRad.push_back(17.45329252f);
    targetAccsRad.push_back(17.45329252f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(34.90658504f);

    mc.setCmd(targetPose, targetVelsRad, targetAccsRad);

    targetPose[0] = 0.0;
    targetPose[1] = 0.0;
    targetPose[2] = 1.5708;
    targetPose[3] = 0.0;
    targetPose[4] = 0.0;
    targetPose[5] = 0.0;
    mc.setCmd(targetPose, targetVelsRad, targetAccsRad);

    double cycleTime = 0.1;

    std::vector<double> cmdPose = curPose;

    for (size_t i=0;i<dof;i++) {
        std::cout << "vertex[" << i << "] = " << curPose[i] << ';' << std::endl;
    }   
    std::cout << std::endl;

    while (mc.execCmd(cycleTime)) { 
        cmdPose = mc.getCmdPose();
        for (size_t i=0;i<dof;i++) {
            std::cout << "vertex[" << i << "] = " << cmdPose[i] << ';' << std::endl;
        }   
        std::cout << std::endl;
    }

    if (mc.execCmd(cycleTime)) return false;

    cmdPose = mc.getCmdPose();

    for (size_t i=0;i<dof;i++) {
        std::cout << "vertex[" << i << "] = " << cmdPose[i] << ';' << std::endl;
    }   
    std::cout << std::endl;

    return true;

}

bool testCase_201(){

    ScurveProfile sc;
    // acc, dec, vel, pos
    if (!sc.makeProf(1, 5, 100)) return false;

    double cycleTime = 0.2;
    double curPos = 0.0;
    double prePos = 0.0;
    double curVel = 0.0;
    double preVel = 0.0;
    double curAcc = 0.0;

    std::string filename = "./testCase/case201.csv";
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return 1;
    }

    std::string line;
    while (std::getline(file, line) && sc.calDis(cycleTime)) { // Read the file line by line
        std::stringstream ss(line);
        std::string value;
        std::vector<std::string> row;

        while (std::getline(ss, value, ',')) {
            row.push_back(value);
        }

        curPos = sc.getCurDis();
        curVel = (curPos - prePos) / cycleTime;
        curAcc = (curVel - preVel) / cycleTime;

        if (abs(curPos - std::stof(row[0])) > 0.0001) {
                return false;
        }
        if (abs(curVel - std::stof(row[1])) > 0.0001) {
                return false;
        }
        if (abs(curAcc - std::stof(row[2])) > 0.0001) {
                return false;
        }

        prePos = curPos;
        preVel = curVel;
    }

    return true;

}

bool testCase_202(){

    ScurveProfile sc;
    // acc, dec, vel, pos
    if(!sc.makeProf(1, 5, 0.25)) return false;

    double cycleTime = 0.2;
    double curPos = 0.0;
    double prePos = 0.0;
    double curVel = 0.0;
    double preVel = 0.0;
    double curAcc = 0.0;

    std::string filename = "./testCase/case202.csv";
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return 1;
    }

    std::string line;
    while (std::getline(file, line) && sc.calDis(cycleTime)) { // Read the file line by line
        std::stringstream ss(line);
        std::string value;
        std::vector<std::string> row;

        while (std::getline(ss, value, ',')) {
            row.push_back(value);
        }

        curPos = sc.getCurDis();
        curVel = (curPos - prePos) / cycleTime;
        curAcc = (curVel - preVel) / cycleTime;

        if (abs(curPos - std::stof(row[0])) > 0.0001) {
                return false;
        }
        if (abs(curVel - std::stof(row[1])) > 0.0001) {
                return false;
        }
        if (abs(curAcc - std::stof(row[2])) > 0.0001) {
                return false;
        }

        prePos = curPos;
        preVel = curVel;
    }

    return true;

}

bool testCase_203(){

    ScurveProfile sc;
    // acc, dec, vel, pos
    if (!sc.makeProf(1, 100, 10)) return false;

    double cycleTime = 0.2;
    double curPos = 0.0;
    double prePos = 0.0;
    double curVel = 0.0;
    double preVel = 0.0;
    double curAcc = 0.0;

    std::string filename = "./testCase/case203.csv";
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return 1;
    }

    std::string line;
    while (std::getline(file, line) && sc.calDis(cycleTime)) { // Read the file line by line
        std::stringstream ss(line);
        std::string value;
        std::vector<std::string> row;

        while (std::getline(ss, value, ',')) {
            row.push_back(value);
        }

        curPos = sc.getCurDis();
        curVel = (curPos - prePos) / cycleTime;
        curAcc = (curVel - preVel) / cycleTime;

        if (abs(curPos - std::stof(row[0])) > 0.0001) {
                return false;
        }
        if (abs(curVel - std::stof(row[1])) > 0.0001) {
                return false;
        }
        if (abs(curAcc - std::stof(row[2])) > 0.0001) {
                return false;
        }

        prePos = curPos;
        preVel = curVel;
    }

    return true;

}

bool testCase301(void){
        
    int dof = 6;
    MotionController mc(dof);

    ScurveProfile* execPorf1 = new ScurveProfile();
    mc.setScurveProfile(execPorf1);
    ScurveProfile* execPorf2 = new ScurveProfile();
    mc.setScurveProfile(execPorf2);

    std::vector<double> curPose;
    curPose.push_back(1.5708f);
    curPose.push_back(-3.14159f);
    curPose.push_back(1.5708f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);
    curPose.push_back(0.0f);

    mc.setCurrentPose(curPose);

    std::vector<double> targetPose;

    targetPose.push_back(0.0f);
    targetPose.push_back(0.0f);
    targetPose.push_back(1.5708f);
    targetPose.push_back(0.0f);
    targetPose.push_back(0.0f);
    targetPose.push_back(0.0f);

    std::vector<double> targetVelsRad;
    targetVelsRad.push_back(5.23598776f);
    targetVelsRad.push_back(5.23598776f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(6.54498469f);
    targetVelsRad.push_back(10.47197551f);

    std::vector<double> targetAccsRad;
    targetAccsRad.push_back(17.45329252f);
    targetAccsRad.push_back(17.45329252f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(26.17993878f);
    targetAccsRad.push_back(34.90658504f);

    mc.setCmd(targetPose, targetVelsRad, targetAccsRad);

    targetPose[0] = 0.0;
    targetPose[1] = 0.0;
    targetPose[2] = 0.0;
    targetPose[3] = 0.0;
    targetPose[4] = 0.0;
    targetPose[5] = 0.0;
    
    mc.setCmd(targetPose, targetVelsRad, targetAccsRad);

    double cycleTime = 0.01;

    std::vector<double> cmdPose = curPose;
    std::vector<double> prePose = curPose;
    std::vector<double> cmdVels;
    cmdVels.push_back(0.0f);
    cmdVels.push_back(0.0f);
    cmdVels.push_back(0.0f);
    cmdVels.push_back(0.0f);
    cmdVels.push_back(0.0f);
    cmdVels.push_back(0.0f);

    std::vector<double> preVels = cmdVels;
    std::vector<double> cmdAccs = cmdVels;

    std::string filenamep = "./testCase/case301_pose.csv";
    //std::string filenamev = "./testCase/case301_vels.csv";
    //std::string filenamea = "./testCase/case301_accs.csv";

    std::ifstream filep(filenamep);
    //std::ifstream filev(filenamev);
    //std::ifstream filea(filenamea);

    if (!filep.is_open()) {
        std::cerr << "Error: Could not open file " << std::endl;
        return false;
    }

/*
    std::ofstream foutp;
    std::ofstream foutv;
    std::ofstream fouta;

    foutp.open(filenamep);
    foutv.open(filenamev);
    fouta.open(filenamea);
*/
    std::string line;
    while (std::getline(filep, line) && mc.execCmd(cycleTime)) { // Read the file line by line
        std::stringstream ss(line);
        std::string value;
        std::vector<std::string> row;

        while (std::getline(ss, value, ',')) {
            row.push_back(value);
        }

        cmdPose = mc.getCmdPose();

        for (size_t i=0;i<dof;i++) {
            if (abs(cmdPose[i] - std::stof(row[i])) > 0.0001) return false;
        }

        for (size_t i=0;i<dof;i++) {
            cmdVels[i] = (cmdPose[i] - prePose[i]) / cycleTime;
            cmdAccs[i] = (cmdVels[i] - preVels[i]) / cycleTime;

            //std::cout << cmdPose[i] << ",";
            //foutp << cmdPose[i] << ",";
            //foutv << cmdVels[i] << ",";
            //fouta << cmdAccs[i] << ",";

            prePose[i] = cmdPose[i];
            preVels[i] = cmdVels[i];
        }
        //std::cout << std::endl;
        //foutp << std::endl;
        //foutv << std::endl;
        //fouta << std::endl;

    }

    cmdPose = mc.getCmdPose();

    for (size_t i=0;i<dof;i++) {
        if (abs(cmdPose[i] - targetPose[i]) > 0.0001) return false;
    }

    return true;
}

int main(){

    int count = 0;

    count++;
    if (testCase01()) std::cout << "PATH:" << count << std::endl;
    else std::cout << "NG:" << count << std::endl;

    count++;
    if (testCase02()) std::cout << "PATH:" << count << std::endl;
    else std::cout << "NG:" << count << std::endl;

    count = 101;
    if (testCase101()) std::cout << "PATH:" << count << std::endl;
    else std::cout << "NG:" << count << std::endl;

    count++;
    if (testCase102()) std::cout << "PATH:" << count << std::endl;
    else std::cout << "NG:" << count << std::endl;

    count = 109;
    if (testCase109()) std::cout << "PATH:" << count << std::endl;
    else std::cout << "NG:" << count << std::endl;

//    testCaseXXX()

    count = 201;
    if (testCase_201()) std::cout << "PATH:" << count << std::endl;
    else std::cout << "NG:" << count << std::endl;

    count++;
    if (testCase_202()) std::cout << "PATH:" << count << std::endl;
    else std::cout << "NG:" << count << std::endl;

    count++;
    if (testCase_203()) std::cout << "PATH:" << count << std::endl;
    else std::cout << "NG:" << count << std::endl;

    if (testCase301()) std::cout << "PATH:" << count << std::endl;
    else std::cout << "NG:" << count << std::endl;

}