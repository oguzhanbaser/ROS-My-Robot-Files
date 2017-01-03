#ifndef my_server_PID_HPP_
#define my_server_PID_HPP_

#include <iostream>
#include <string.h>

namespace my_server {

class PID
{
public:
    PID();
    ~PID();
    int compute(double val);
    void setConstants(double _kp, double _kd, double _ki);
    void setConstants(std::string name, double val);
    int getPIDVal();
    void setLastTime();
    int getControlSignal();
    double *getDebugVals();
    void resetValues();

    double limitIn, limitControlSignal;
    int limitPID;

private:
    double debugData[5];
    double limitDer, limitPr, PIDval,kp, kd, ki, pr, time, lastError, error, der, in;
    unsigned long int lastTime , timeNow;
    int controlSignal, lastControlSignal;
    int noiseLevel;
};

}

#endif
