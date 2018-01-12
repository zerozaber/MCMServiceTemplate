#include "HardwareLibrary.h"
#include "MCMServiceConfig.h"
#include "MCMService.h"
byte count = 0;
byte count2 = 0;
void setup(){
  pSerial1.begin(115200); //TX1 RX1, polling mode by mcmService
  pSerial2.begin(115200); //TX2 RX2, polling mode by mcmService
  MCMServiceInitialize();
  mcmService.runInService = realTimeFunction;
  MCMServiceStartService();
}

void loop(){
  pSerial1.println(count);
  if(count == 0)
    count = 1;
  else
    count = 0;
}
void realTimeFunction(){
  pSerial2.println(count2);
  if(count2 == 0)
    count2 = 1;
  else
    count2 = 0;
}

