#include "MCMService.h"
#include "HardwareLibrary.h"
#include "MCMServiceConfig.h"
#define BaseAddress (0xfe00)
#include "Arduino.h"
void MCMServiceInitialize(){
  HardwareInitialize();//initialize hardware
  delay(10);
  #ifdef MOTOR_0_USED
    PwmInitialize(0, MOTOR_0_PWM_CLOCK);
  #endif
  #ifdef MOTOR_1_USED
    PwmInitialize(1, MOTOR_1_PWM_CLOCK);
  #endif
  #ifdef MOTOR_2_USED
    PwmInitialize(2, MOTOR_2_PWM_CLOCK);
  #endif
  #ifdef MOTOR_3_USED
    PwmInitialize(3, MOTOR_3_PWM_CLOCK);
  #endif
  #ifdef MOTOR_4_USED
    PwmInitialize(4, MOTOR_4_PWM_CLOCK);
  #endif
  #ifdef MOTOR_5_USED
    PwmInitialize(5, MOTOR_5_PWM_CLOCK);
  #endif
  PwmSetSampleCycle(SERVO_MAIN_INTERRUPT, SERVO_RATE_SCALER);
  #ifdef ENCODER_0_USED
    Enc[0].begin(ENCODER_0_MODE);
    mcmService.useEncoder[0] = true;
    Enc[0].write(0);
  #endif
  #ifdef ENCODER_1_USED
    Enc[1].begin(ENCODER_1_MODE);
    mcmService.useEncoder[1] = true;
    Enc[1].write(0);
  #endif
  #ifdef ENCODER_2_USED
    Enc[2].begin(ENCODER_2_MODE);
    mcmService.useEncoder[2] = true;
    Enc[2].write(0);
  #endif
  #ifdef ENCODER_3_USED
    Enc[3].begin(ENCODER_3_MODE);
    mcmService.useEncoder[3] = true;
    Enc[3].write(0);
  #endif
  #ifdef ENCODER_4_USED
    Enc[4].begin(ENCODER_4_MODE);
    mcmService.useEncoder[4] = true;
    Enc[4].write(0);
  #endif
  #ifdef ENCODER_5_USED
    Enc[5].begin(ENCODER_5_MODE);
    mcmService.useEncoder[5] = true;
    Enc[5].write(0);
  #endif
  mcmService.ADCEnable = 0;
  #ifdef ADC_0_USED
    mcmService.ADCEnable += 1L<<0;
  #endif
  #ifdef ADC_1_USED
    mcmService.ADCEnable += 1L<<1;
  #endif
  #ifdef ADC_2_USED
    mcmService.ADCEnable += 1L<<2;
  #endif
  #ifdef ADC_3_USED
    mcmService.ADCEnable += 1L<<3;
  #endif
  #ifdef ADC_4_USED
    mcmService.ADCEnable += 1L<<4;
  #endif
  #ifdef ADC_5_USED
    mcmService.ADCEnable += 1L<<5;
  #endif
  #ifdef ADC_6_USED
    mcmService.ADCEnable += 1L<<6;
  #endif
  #ifdef ADC_7_USED
    mcmService.ADCEnable += 1L<<7;
  #endif
  
  #ifdef MOTOR_0_USED
    mcmService.useMotor[0] = true;
  #endif
  #ifdef MOTOR_1_USED
    mcmService.useMotor[1] = true;
  #endif
  #ifdef MOTOR_2_USED
    mcmService.useMotor[2] = true;
  #endif
  #ifdef MOTOR_3_USED
    mcmService.useMotor[3] = true;
  #endif
  #ifdef MOTOR_4_USED
    mcmService.useMotor[4] = true;
  #endif
  #ifdef MOTOR_5_USED
    mcmService.useMotor[5] = true;
  #endif
  
  
  io_DisableINT();
  io_outpb(BaseAddress + 1, 0x08);//disable ADC
  io_outpb(BaseAddress + 0, mcmService.ADCEnable);//enable scanning
  io_outpb(BaseAddress + 1, 0x01);//start with auto scan mode
  io_RestoreINT();
}
void MCMServiceStartService(){
  attachPwmInterrupt(MCMServiceMainFunction, SERVO_MAIN_INTERRUPT);
}
void MCMServiceMainFunction(){
  mcmService.realServiceFunction();
}
MCMService mcmService;
MCMService::MCMService(){
}

void MCMService::realServiceFunction(){
  count++;
  deltaT2 = micros()-t;
  t = micros();
  setPWM();
  getDirVal();
  unsafeSetDirectionAll(dirVal);
  readEncoder();
  #ifdef READ_LIMIT_SWITCH
    readLimitSwitch();
  #endif
  readAnalog();
  if(runInService != NULL)
    runInService();
  #ifdef USE_AUX_DIR_STEP_0
    runAuxDirStep0();
  #endif
  pSerial1.poll(UART_MAX_TRANSMIT_PER_LOOP, UART_MAX_RECEIVE_PER_LOOP);
  pSerial2.poll(UART_MAX_TRANSMIT_PER_LOOP, UART_MAX_RECEIVE_PER_LOOP);
  pSerial3.poll(UART_MAX_TRANSMIT_PER_LOOP, UART_MAX_RECEIVE_PER_LOOP);
  deltaT = micros()-t;
  if(deltaT > maxDeltaT) maxDeltaT = deltaT;
}
#ifdef USE_AUX_DIR_STEP_0
void MCMService::runAuxDirStep0(){
  stepCount = 0;
  while(true){
    if(stepPinStatus){//if step is still high, pull it down
      stepPinStatus = false;
      unsafeDigitalWriteCustomPin(PIN_DIR_7, stepPinStatus);
      if(dirPinStatus)
        dirStepAuxBuffer0 += 1;
      else
        dirStepAuxBuffer0 -= 1;
      delayMicroseconds(AUX_DIR_STEP_0_DELAY);
      stepCount++;
      if(stepCount == AUX_DIR_STEP_0_MAX_PULSE_PER_LOOP) break;
    }
    if(dirStepAuxBuffer0 == 0) break;
    if(dirStepAuxBuffer0 > 0){
      if(dirPinStatus){//if dir pin is high, pull it down
        dirPinStatus = false;
        unsafeDigitalWriteCustomPin(PIN_DIR_6, dirPinStatus);
        delayMicroseconds(AUX_DIR_STEP_0_DELAY);
      }
      stepPinStatus = true;
      unsafeDigitalWriteCustomPin(PIN_DIR_7, stepPinStatus);
      delayMicroseconds(AUX_DIR_STEP_0_DELAY);
    }else{
      if(!dirPinStatus){//if dir pin is high, pull it down
        dirPinStatus = true;
        unsafeDigitalWriteCustomPin(PIN_DIR_6, dirPinStatus);
        delayMicroseconds(AUX_DIR_STEP_0_DELAY);
      }
      stepPinStatus = true;
      unsafeDigitalWriteCustomPin(PIN_DIR_7, stepPinStatus);
      delayMicroseconds(AUX_DIR_STEP_0_DELAY);
    }
  }
  return;
}
#endif
void MCMService::getDirVal(){
  dirVal = 0;
  #if MOTOR_0_TYPE == 0
    if(!signbit(motorPower[0]))
      dirVal += 1<<0;
  #endif
  #if MOTOR_0_TYPE == 1
    if(motorEnable[0])
      dirVal += 1<<0;
  #endif
  
  #if MOTOR_1_TYPE == 0
    if(!signbit(motorPower[1]))
      dirVal += 1<<1;
  #endif
  #if MOTOR_1_TYPE == 1
    if(motorEnable[1])
      dirVal += 1<<1;
  #endif
  
  #if MOTOR_2_TYPE == 0
    if(!signbit(motorPower[2]))
      dirVal += 1<<2;
  #endif
  #if MOTOR_2_TYPE == 1
    if(motorEnable[2])
      dirVal += 1<<2;
  #endif
  
  #if MOTOR_3_TYPE == 0
    if(!signbit(motorPower[3]))
      dirVal += 1<<3;
  #endif
  #if MOTOR_3_TYPE == 1
    if(motorEnable[3])
      dirVal += 1<<3;
  #endif
  
  #if MOTOR_4_TYPE == 0
    if(!signbit(motorPower[4]))
      dirVal += 1<<4;
  #endif
  #if MOTOR_4_TYPE == 1
    if(motorEnable[4])
      dirVal += 1<<4;
  #endif
  
  #if MOTOR_5_TYPE == 0
    if(!signbit(motorPower[5]))
      dirVal += 1<<5;
  #endif
  #if MOTOR_5_TYPE == 1
    if(motorEnable[5])
      dirVal += 1<<5;
  #endif
  
}
void MCMService::setPWM(){
  
  #ifdef MOTOR_0_USED
    #if MOTOR_0_TYPE == 0 //sign-magnitude
      if(!motorEnable[0])
        unsafeSetPwm(0, 0, 0, MOTOR_0_PWM_CLOCK);
      else if(signbit(motorPower[0]))//negative
        unsafeSetPwm(0, 0, -motorPower[0]*MOTOR_0_PWM_CLOCK, MOTOR_0_PWM_CLOCK);//because getDirVal has been called before
      else
        unsafeSetPwm(0, 0, motorPower[0]*MOTOR_0_PWM_CLOCK, MOTOR_0_PWM_CLOCK);//because getDirVal has been called before
      
    #endif
    #if MOTOR_0_TYPE == 1 //locked anti-phase
      unsafeSetPwm(0, 0, (0.5+(motorPower[0]/2.0))*MOTOR_0_PWM_CLOCK, MOTOR_0_PWM_CLOCK);
    #endif
  #endif
  
  #ifdef MOTOR_1_USED
    #if MOTOR_1_TYPE == 0 //sign-magnitude
      if(!motorEnable[1])
        unsafeSetPwm(0, 1, 0, MOTOR_1_PWM_CLOCK);
      else if(signbit(motorPower[1]))//negative
        unsafeSetPwm(0, 1, -motorPower[1]*MOTOR_1_PWM_CLOCK, MOTOR_1_PWM_CLOCK);//because getDirVal has been called before
      else
        unsafeSetPwm(0, 1, motorPower[1]*MOTOR_1_PWM_CLOCK, MOTOR_1_PWM_CLOCK);//because getDirVal has been called before
      
    #endif
    #if MOTOR_1_TYPE == 1 //locked anti-phase
      unsafeSetPwm(0, 1, (0.5+(motorPower[1]/2.0))*MOTOR_1_PWM_CLOCK, MOTOR_1_PWM_CLOCK);
    #endif
  #endif
  
  #ifdef MOTOR_2_USED
    #if MOTOR_2_TYPE == 0 //sign-magnitude
      if(!motorEnable[2])
        unsafeSetPwm(0, 2, 0, MOTOR_2_PWM_CLOCK);
      else if(signbit(motorPower[2]))//negative
        unsafeSetPwm(0, 2, -motorPower[2]*MOTOR_2_PWM_CLOCK, MOTOR_2_PWM_CLOCK);//because getDirVal has been called before
      else
        unsafeSetPwm(0, 2, motorPower[2]*MOTOR_2_PWM_CLOCK, MOTOR_2_PWM_CLOCK);//because getDirVal has been called before
      
    #endif
    #if MOTOR_2_TYPE == 1 //locked anti-phase
      unsafeSetPwm(0, 2, (0.5+(motorPower[2]/2.0))*MOTOR_2_PWM_CLOCK, MOTOR_2_PWM_CLOCK);
    #endif
  #endif
  
  #ifdef MOTOR_3_USED
    #if MOTOR_3_TYPE == 0 //sign-magnitude
      if(!motorEnable[3])
        unsafeSetPwm(1, 0, 0, MOTOR_3_PWM_CLOCK);
      else if(signbit(motorPower[3]))//negative
        unsafeSetPwm(1, 0, -motorPower[3]*MOTOR_3_PWM_CLOCK, MOTOR_3_PWM_CLOCK);//because getDirVal has been called before
      else
        unsafeSetPwm(1, 0, motorPower[3]*MOTOR_3_PWM_CLOCK, MOTOR_3_PWM_CLOCK);//because getDirVal has been called before
      
    #endif
    #if MOTOR_3_TYPE == 1 //locked anti-phase
      unsafeSetPwm(1, 0, (0.5+(motorPower[3]/2.0))*MOTOR_3_PWM_CLOCK, MOTOR_3_PWM_CLOCK);
    #endif
  #endif
  
  #ifdef MOTOR_4_USED
    #if MOTOR_4_TYPE == 0 //sign-magnitude
      if(!motorEnable[4])
        unsafeSetPwm(1, 1, 0, MOTOR_4_PWM_CLOCK);
      else if(signbit(motorPower[4]))//negative
        unsafeSetPwm(1, 1, -motorPower[4]*MOTOR_4_PWM_CLOCK, MOTOR_4_PWM_CLOCK);//because getDirVal has been called before
      else
        unsafeSetPwm(1, 1, motorPower[4]*MOTOR_4_PWM_CLOCK, MOTOR_4_PWM_CLOCK);//because getDirVal has been called before
      
    #endif
    #if MOTOR_4_TYPE == 1 //locked anti-phase
      unsafeSetPwm(1, 1, (0.5+(motorPower[4]/2.0))*MOTOR_4_PWM_CLOCK, MOTOR_4_PWM_CLOCK);
    #endif
  #endif
  
  #ifdef MOTOR_5_USED
    #if MOTOR_5_TYPE == 0 //sign-magnitude
      if(!motorEnable[5])
        unsafeSetPwm(1, 2, 0, MOTOR_5_PWM_CLOCK);
      else if(signbit(motorPower[5]))//negative
        unsafeSetPwm(1, 2, -motorPower[5]*MOTOR_5_PWM_CLOCK, MOTOR_5_PWM_CLOCK);//because getDirVal has been called before
      else
        unsafeSetPwm(1, 2, motorPower[5]*MOTOR_5_PWM_CLOCK, MOTOR_5_PWM_CLOCK);//because getDirVal has been called before
      
    #endif
    #if MOTOR_5_TYPE == 1 //locked anti-phase
      unsafeSetPwm(1, 2, (0.5+(motorPower[5]/2.0))*MOTOR_5_PWM_CLOCK, MOTOR_5_PWM_CLOCK);
    #endif
  #endif
}
void MCMService::readEncoder(){
  #ifdef ENCODER_0_USED
    encoderPosition[0] = Enc[0].read();
  #endif
  
  #ifdef ENCODER_1_USED
    encoderPosition[1] = Enc[1].read();
  #endif
  
  #ifdef ENCODER_2_USED
    encoderPosition[2] = Enc[2].read();
  #endif
  
  #ifdef ENCODER_3_USED
    encoderPosition[3] = Enc[3].read();
  #endif
  
  #ifdef ENCODER_4_USED
    encoderPosition[4] = Enc[4].read();
  #endif
  
  #ifdef ENCODER_5_USED
    encoderPosition[5] = Enc[5].read();
  #endif
}
void MCMService::readAnalog(){
  while((io_inpb(BaseAddress+2) & 0x01) != 0){
    tempAnalogValue = io_inpw(BaseAddress + 4);
    analogValue[(tempAnalogValue & 0xe000)>>13] = tempAnalogValue & 0x07ff;
  }
  io_outpb(BaseAddress + 1, 0x08);//disable ADC
  io_outpb(BaseAddress + 0, mcmService.ADCEnable);//enable scanning
  io_outpb(BaseAddress + 1, 0x01);//start with auto scan mode
}
void MCMService::disableAllMotor(){
  motorPower[0] = 0;
  motorPower[1] = 0;
  motorPower[2] = 0;
  motorPower[3] = 0;
  motorPower[4] = 0;
  motorPower[5] = 0;
  setPWM();
  getDirVal();
  unsafeSetDirectionAll(dirVal);
}
void MCMService::readLimitSwitch(){
  byte lmByte;
  lmByte = unsafeReadLimitSwitchAll();
  for(int i = 0; i<6; i++){
    if ((lmByte & 1<<i) != 0)
      lmStatus[i] = true;
    else
      lmStatus[i] = false;
  }
}

