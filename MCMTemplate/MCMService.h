#ifndef MCMService_h
#define MCMService_h
#include "MCMServiceConfig.h"
#include "HardwareLibrary.h"
#include <stdint.h>
void MCMServiceInitialize();//this include initialization of hardware library
void MCMServiceStartService();
void MCMServiceMainFunction();
class MCMService{
  public:
    MCMService();
    void realServiceFunction();
    void (*runInService)();
    volatile unsigned short count = 0;
    volatile int encoderPosition[6] = {0, 0, 0, 0, 0, 0};//Read-only. This variable holds encoder value read in service.
    volatile float motorPower[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    volatile bool motorEnable[6] = {true, true, true, true, true, true};
    volatile unsigned int deltaT = 0;
    volatile unsigned int maxDeltaT = 0;
    volatile unsigned int deltaT2 = 0;
    volatile unsigned short analogValue[8];
    volatile bool lmStatus[6] = {false, false, false, false, false, false};
    volatile short dirStepAuxBuffer0 = 0;//to manipulate this, call io_DisableINT(); then increase or decrease this value, follow by io_RestoreINT(); Note that this value is relative, not absolute.
    
    //do not access variables below this
    volatile byte dirVal = 0;
    volatile unsigned int t = 0;
    volatile byte ADCEnable = 0;
    volatile bool useMotor[6] = {false, false, false, false, false, false};
    volatile bool useEncoder[6] = {false, false, false, false, false, false};
  private:
    
    void readEncoder();
    void getDirVal();
    void setPWM();
    void readAnalog();
    void disableAllMotor();
    void readLimitSwitch();
    
    volatile unsigned short tempAnalogValue;//temp analog value for reading
    #ifdef USE_AUX_DIR_STEP_0
      volatile bool dirPinStatus = false;
      volatile bool stepPinStatus = false;
      volatile unsigned short stepCount = 0;
      void runAuxDirStep0();
    #endif
    
};
extern MCMService mcmService;
#endif
