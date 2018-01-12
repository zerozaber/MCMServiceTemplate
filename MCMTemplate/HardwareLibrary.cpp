#include "HardwareLibrary.h"
#include <stdio.h>
#include "io.h"
#include "mcm.h"
#include "mcm.cpp"
#include "irq.h"
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "Stream.h"
#include "com.h"
#include "HardwareSerial.h"


#if defined (__86DUINO_AI)

  #define  COM1_TX    (0x9C)
  #define  COM1_RX    (0x9D)
  #define  COM2_TX    (0x9E)
  #define  COM2_RX    (0x9F)
  #define  COM3_TX    (0x98)
  #define  COM3_RX    (0x99)

#else
  
  #define  COM1_TX    (0x9A)
  #define  COM1_RX    (0x9B)
  #define  COM2_TX    (0x9E)
  #define  COM2_RX    (0x9F)
  #define  COM3_TX    (0x9C)
  #define  COM3_RX    (0x9D)

#endif

PollingSerial::PollingSerial(int _number, unsigned long com_baudrate, unsigned char com_format, unsigned long com_rxtimeout, unsigned long com_txtimeout) {
  number      = _number;
  baudrate    = com_baudrate;
  format      = com_format;
  rxtimeout   = com_rxtimeout;
  txtimeout   = com_txtimeout;
  peek_stored = false;
}

void PollingSerial::begin(unsigned long baud) {
  begin(baud, format, COM_FullDuplex);
}
void PollingSerial::begin(unsigned long baud, int comtype) {
  begin(baud, format, comtype);
}
void PollingSerial::begin(unsigned long baud, uint8_t config) {
    begin(baud, config, COM_FullDuplex);
}
void PollingSerial::begin(unsigned long baud, uint8_t config, int comtype) {
  sb_Write(0xc0, (sb_Read(0xc0) & 0x7fffffffL) | ((unsigned long)1L << 31));
  io_Close();
  switch (baud) {
    case 6000000L: baud = COM_UARTBAUD_6000000BPS; break;
    case 3000000L: baud = COM_UARTBAUD_3000000BPS; break;
    case 2000000L: baud = COM_UARTBAUD_2000000BPS; break;
    case 1500000L: baud = COM_UARTBAUD_1500000BPS; break;
    case 1000000L: baud = COM_UARTBAUD_1000000BPS; break;
    case 750000L:  baud = COM_UARTBAUD_750000BPS;  break;
    case 500000L:  baud = COM_UARTBAUD_500000BPS;  break;
    case 461538L:  baud = COM_UARTBAUD_461538BPS;  break;
    case 333333L:  baud = COM_UARTBAUD_333333BPS;  break;
    case 300000L:  baud = COM_UARTBAUD_300000BPS;  break;
    case 250000L:  baud = COM_UARTBAUD_250000BPS;  break;
    case 200000L:  baud = COM_UARTBAUD_200000BPS;  break;
    case 150000L:  baud = COM_UARTBAUD_150000BPS;  break;
    case 125000L:  baud = COM_UARTBAUD_125000BPS;  break;
    case 115200L:  baud = COM_UARTBAUD_115200BPS;  break;
    case 57600L:   baud = COM_UARTBAUD_57600BPS;   break;
    case 38400L:   baud = COM_UARTBAUD_38400BPS;   break;
    case 28800L:   baud = COM_UARTBAUD_28800BPS;   break;
    case 19200L:   baud = COM_UARTBAUD_19200BPS;   break;
    case 14400L:   baud = COM_UARTBAUD_14400BPS;   break;
    case 9600L:    baud = COM_UARTBAUD_9600BPS;    break;
    case 4800L:    baud = COM_UARTBAUD_4800BPS;    break;
    case 2400L:    baud = COM_UARTBAUD_2400BPS;    break;
    case 1200L:    baud = COM_UARTBAUD_1200BPS;    break;
    case 600L:     baud = COM_UARTBAUD_600BPS;     break;
    case 300L:     baud = COM_UARTBAUD_300BPS;     break;
    case 50L:      baud = COM_UARTBAUD_50BPS;      break;
    default:       baud = COM_UARTBAUD_9600BPS;    break;
  }
  //Now its time to set BPS, Format, reset buffer
  unsigned short divisor;
  if(vx86_uart_GetSBCLK() != 0)
    divisor = (unsigned short)(baud >> 16);
  else
    divisor = (unsigned short)(baud);
  unsigned long configValue = 0x00800000L | (((unsigned long)(divisor & 0x8000)) << 7) | 0x00000000L | (((unsigned long)(divisor & 0x4000)) << 6);//UE = 1, set CS, FIFO size = 16, set HCS
  io_outpdw(uartConfigBaseAddress, (io_inpdw(uartConfigBaseAddress) & 0x0000FFFFL) | configValue);
  io_outpb(uartBaseAddress+3, 0x80 | config); //turn DLAB on
  divisor = divisor & 0x3FFF;
  io_outpb(uartBaseAddress, (byte)(divisor & 0x00FF));
  io_outpb(uartBaseAddress+1, (byte)((divisor & 0xFF00) >> 8));
  io_outpb(uartBaseAddress+3, config); //turn DLAB off
  io_outpb(uartBaseAddress+1, 0); //no interrupt
  io_outpb(uartBaseAddress+4, 0x03);
  //turnoff FIFO
  io_outpb(uartBaseAddress+2, 0);
  //turnon FIFO
  io_outpb(uartBaseAddress+2, 1);
  initialized = true;
  
}
int PollingSerial::available(){
  return bufferNotRead;
}
int PollingSerial::peek(void){
  return receivingBuffer[receivingBufferPosition];
}
int PollingSerial::read(void){
  if(!initialized)
    return 0;
  if(bufferNotRead == 0)
    return 0;
  io_DisableINT();
  byte tempC = receivingBuffer[receivingBufferPosition];
  receivingBufferPosition++;
  if(receivingBufferPosition >= POLLING_SERIAL_BUFFER_SIZE)//ring overflow
    receivingBufferPosition = 0;
  bufferNotRead--;
  io_RestoreINT();
  return tempC;
}
void PollingSerial::flush(void){
  if(!initialized)
    return;
  while(bufferNotSent >= 0){;}
  return;
}
size_t PollingSerial::write(uint8_t c){
  if(!initialized)
    return 0;
  if(bufferNotSent >= POLLING_SERIAL_BUFFER_SIZE)
    return 0;
  io_DisableINT();
  sendingBuffer[sendingBufferPosition] = c;
  sendingBufferPosition++;
  if(sendingBufferPosition >= POLLING_SERIAL_BUFFER_SIZE)//ring overflow
    sendingBufferPosition = 0;
  bufferNotSent++;
  io_RestoreINT();
  return 1;
}
void PollingSerial::poll(unsigned long maxTransmitBytes, unsigned long maxReceiveBytes){
  if(!initialized)
    return;
  while(maxTransmitBytes > 0 || maxReceiveBytes > 0){
    volatile byte lineStatus;
    lineStatus = io_inpb(uartBaseAddress+5);
    if((lineStatus & 0x20) != 0 && bufferNotSent > 0 && maxTransmitBytes > 0){//do write
      io_outpb(uartBaseAddress, sendingBuffer[sendingBufferPositionForPolling]);
      sendingBufferPositionForPolling++;
      if(sendingBufferPositionForPolling >= POLLING_SERIAL_BUFFER_SIZE)
        sendingBufferPositionForPolling = 0;
      bufferNotSent--;
      maxTransmitBytes--;
    }else{
      maxTransmitBytes = 0;
    }
    if((lineStatus & 0x01) != 0 && maxReceiveBytes > 0){
      receivingBuffer[receivingBufferPositionForPolling] = io_inpb(uartBaseAddress);
      receivingBufferPositionForPolling++;
      if(receivingBufferPositionForPolling >= POLLING_SERIAL_BUFFER_SIZE)
        receivingBufferPositionForPolling = 0;
      bufferNotRead++;
      maxReceiveBytes--;
    }else{
      maxReceiveBytes = 0;
    }
  }
}
PollingSerial pSerial1(0, 115200L, BYTESIZE8|NOPARITY|STOPBIT1, 0L, 500L);
PollingSerial pSerial2(1, 115200L, BYTESIZE8|NOPARITY|STOPBIT1, 0L, 500L);
PollingSerial pSerial3(2, 115200L, BYTESIZE8|NOPARITY|STOPBIT1, 0L, 500L);

//Hardware
#define SB_CROSSBASE  (0x64)
#define CROSSBARBASE  (0x0A00)
#define SB_FCREG      (0xC0)
#define SB_GPIOBASE   (0x62)
#define GPIOCTRLBASE  (0xF100)
#define GPIODATABASE  (0xF200)
#define GPIODIRBASE   (0xF202)

unsigned int dirPin[6] = {32, 33, 34, 35, 36, 37};
unsigned int dirPinPort = GPIODATABASE+16;

void HardwareInitialize(){
  //set MCM mode
  io_DisableINT();
  mc_outp(MC_GENERAL, MCG_MODEREG1, 0x5533L);// set MCM 3 and 2 to dual sensor, set MCM1, 0 to independent PWM and sensor
  io_RestoreINT();
  //set crossbar
  unsigned short crossbar_ioaddr = sb_Read16(0x64)&0xfffe;
  io_DisableINT();
  io_outpb(crossbar_ioaddr, 0x03);// GPIO_0 = MCM_2
  io_outpb(crossbar_ioaddr + 2, 0x01);// GPIO_2 = MCM_0
  io_outpb(crossbar_ioaddr + 3, 0x02);// GPIO_3 = MCM_1
  io_outpb(crossbar_ioaddr + 0x90 + 17, 0x08);
  io_outpb(crossbar_ioaddr + 0x90 + 19, 0x08);
  io_outpb(crossbar_ioaddr + 0x90 + 20, 0x08);
  io_outpb(crossbar_ioaddr + 0x90 + 25, 0x08);
  io_outpb(crossbar_ioaddr + 0x90 + 27, 0x08);
  io_outpb(crossbar_ioaddr + 0x90 + 28, 0x08);
  //below is open pin for encoder, order by sensor number and pin
  io_outpb(crossbar_ioaddr + 0x90 + 16, 0x08);
  io_outpb(crossbar_ioaddr + 0x90 + 18, 0x08);
  io_outpb(crossbar_ioaddr + 0x90 + 21, 0x08);
  io_outpb(crossbar_ioaddr + 0x90 + 24, 0x08);
  io_outpb(crossbar_ioaddr + 0x90 + 26, 0x08);
  io_outpb(crossbar_ioaddr + 0x90 + 29, 0x08);
  io_outpb(crossbar_ioaddr + 0x90 + 0, 0x08);
  io_outpb(crossbar_ioaddr + 0x90 + 2, 0x08);
  io_outpb(crossbar_ioaddr + 0x90 + 5, 0x08);
  io_outpb(crossbar_ioaddr + 0x90 + 22, 0x08);
  io_outpb(crossbar_ioaddr + 0x90 + 30, 0x08);
  io_outpb(crossbar_ioaddr + 0x90 + 6, 0x08);
  io_outpb(crossbar_ioaddr + 0x90 + 1, 0x08);
  io_outpb(crossbar_ioaddr + 0x90 + 3, 0x08);
  io_outpb(crossbar_ioaddr + 0x90 + 4, 0x08);
  io_outpb(crossbar_ioaddr + 0x90 + 23, 0x08);
  io_outpb(crossbar_ioaddr + 0x90 + 31, 0x08);
  io_outpb(crossbar_ioaddr + 0x90 + 7, 0x08);
  //enable PWM
  mcpwm_Enable(0, 0);//mcn, mdn
  mcpwm_Enable(0, 1);
  mcpwm_Enable(0, 2);
  mcpwm_Enable(1, 0);
  mcpwm_Enable(1, 1);
  mcpwm_Enable(1, 2);
  //finish set crossbar
  io_RestoreINT();
  io_DisableINT();
  //set pinMode of direction control pin
  //check crossbar register
  crossbar_ioaddr = sb_Read16(SB_CROSSBASE) & 0xfffe;
  if(crossbar_ioaddr == 0 || crossbar_ioaddr == 0xfffe){
    sb_Write16(SB_CROSSBASE, CROSSBARBASE | 0x01);
    crossbar_ioaddr = CROSSBARBASE;
  }
  //check gpio register
  unsigned short gpio_ioaddr = sb_Read16(SB_GPIOBASE) & 0xfffe;
  if(gpio_ioaddr == 0 || gpio_ioaddr == 0xfffe){
    sb_Write16(SB_GPIOBASE, GPIOCTRLBASE | 0x01);
    gpio_ioaddr = GPIOCTRLBASE;
  }
  io_outpdw(gpio_ioaddr, 0x00ff);
  for(int i=0;i<8;i++)
    io_outpdw(gpio_ioaddr + (i+1)*4,((GPIODIRBASE + i*4)<<16) + GPIODATABASE + i*4);
  //set pinMode of DIR pins
  io_outpb(GPIODIRBASE + 16, 0xff);
  //set output of DIR pins to low
  io_outpb(GPIODATABASE + 16, 0);
  //set limit switch input, only 6 pin
  for(int i = 0; i<6; i++){
    io_outpb(crossbar_ioaddr + 0x30 + 40 + i, 0x00);//no pullup or pulldown
    io_outpb(GPIODIRBASE + 4*(5), io_inpb(GPIODIRBASE + 4*(5))&~(1<<(i)));
  }
  io_RestoreINT();
  //Set Polling Serial
  io_Init();
  io_DisableINT();
  unsigned short tempUartBaseAddress = sb_Read16(0x60) & 0xfffe;
  sb_Write16(0x60, sb_Read16(0x60) | 0x0001);
  unsigned long cfg_data;
  cfg_data = io_inpdw(tempUartBaseAddress);
  if((cfg_data & 0x00800000L) != 0L)
    pSerial1.uartBaseAddress = (unsigned short)(cfg_data & 0x0000FFFFL);
    pSerial1.uartConfigBaseAddress = tempUartBaseAddress;
  cfg_data = io_inpdw(tempUartBaseAddress + 4);
  if((cfg_data & 0x00800000L) != 0L)
    pSerial2.uartBaseAddress = (unsigned short)(cfg_data & 0x0000FFFFL);
    pSerial2.uartConfigBaseAddress = tempUartBaseAddress+4;
  cfg_data = io_inpdw(tempUartBaseAddress + 8);
  if((cfg_data & 0x00800000L) != 0L)
    pSerial3.uartBaseAddress = (unsigned short)(cfg_data & 0x0000FFFFL);
    pSerial3.uartConfigBaseAddress = tempUartBaseAddress+8;
  io_outpb(crossbar_ioaddr + COM1_TX, 0x08);
  io_outpb(crossbar_ioaddr + COM1_RX, 0x08);
  io_outpb(crossbar_ioaddr + COM2_TX, 0x08);
  io_outpb(crossbar_ioaddr + COM2_RX, 0x08);
  io_outpb(crossbar_ioaddr + COM3_TX, 0x08);
  io_outpb(crossbar_ioaddr + COM3_RX, 0x08);
  sb_Write(0xc0, (sb_Read(0xc0) | 0x80000000L));//set SBCLK
  io_RestoreINT();
}
void PwmInitialize(unsigned int slot, unsigned int _period){
  unsigned int _mc;
  unsigned int _md;
  _mc = slot/3;
  _md = slot%3;
  io_DisableINT();
  mcpwm_SetWidth(_mc, _md, _period-1, _period-1);
  mcpwm_ReloadPWM(_mc, _md, MCPWM_RELOAD_CANCEL);
  mcpwm_SetOutMask(_mc, _md, MCPWM_HMASK_NONE + MCPWM_LMASK_NONE);
  mcpwm_SetOutPolarity(_mc, _md, MCPWM_HPOL_NORMAL + MCPWM_LPOL_NORMAL);
  mcpwm_SetDeadband(_mc, _md, 0L);
  mcpwm_ReloadOUT_Unsafe(_mc, _md, MCPWM_RELOAD_NOW);
  mcpwm_SetWaveform(_mc, _md, MCPWM_EDGE_I0A1);//active first, then inactive
  mcpwm_SetSamplCycle(_mc, _md, 3L); // sample cycle: 20ms
  io_RestoreINT();
  io_DisableINT();
  mcpwm_SetWidth(_mc, _md, _period-1, 0);
  mcpwm_ReloadPWM(_mc, _md, MCPWM_RELOAD_PEREND);
  io_RestoreINT();
}
void PwmInitializeWithSampleCycle(unsigned int slot, unsigned int _period, unsigned int _sc){
  unsigned int _mc;
  unsigned int _md;
  _mc = slot/3;
  _md = slot%3;
  io_DisableINT();
  mcpwm_SetWidth(_mc, _md, _period-1, _period-1);
  mcpwm_ReloadPWM(_mc, _md, MCPWM_RELOAD_CANCEL);
  mcpwm_SetOutMask(_mc, _md, MCPWM_HMASK_NONE + MCPWM_LMASK_NONE);
  mcpwm_SetOutPolarity(_mc, _md, MCPWM_HPOL_NORMAL + MCPWM_LPOL_NORMAL);
  mcpwm_SetDeadband(_mc, _md, 0L);
  mcpwm_ReloadOUT_Unsafe(_mc, _md, MCPWM_RELOAD_NOW);
  mcpwm_SetWaveform(_mc, _md, MCPWM_EDGE_I0A1);//active first, then inactive
  mcpwm_SetSamplCycle(_mc, _md, _sc-1); // sample cycle: 20ms
  io_RestoreINT();
  io_DisableINT();
  mcpwm_SetWidth(_mc, _md, _period-1, 0);
  mcpwm_ReloadPWM(_mc, _md, MCPWM_RELOAD_PEREND);
  io_RestoreINT();
  
}
void PwmSetSampleCycle(unsigned int slot, unsigned int _sc){
  unsigned int _mc;
  unsigned int _md;
  _mc = slot/3;
  _md = slot%3;
  io_DisableINT();
  mcpwm_SetSamplCycle(_mc, _md, _sc-1);
  io_RestoreINT();
}
void safeSetPwm(unsigned int i, unsigned int j, unsigned int _duty, unsigned int _period){
  io_DisableINT();
  unsafeSetPwm(i, j, _duty, _period);
  io_RestoreINT();
}
void unsafeSetPwm(unsigned int i, unsigned int j, unsigned int _duty, unsigned int _period){
  if(_duty < 1) _duty = 1;
  else if(_duty > _period) _duty = _period;
  mcpwm_SetWidth(i, j, _period-1, _period-_duty);
  mcpwm_ReloadPWM(i, j, MCPWM_RELOAD_NOW);
}
void setDirection(unsigned int slot, bool _dir){
  io_DisableINT();
  unsafeSetDirection(slot, _dir);
  io_RestoreINT();
}
void unsafeSetDirection(unsigned int slot, bool _dir){
  if(_dir)
    io_outpb(dirPinPort, io_inpb(dirPinPort)|(1<<slot));
  else
    io_outpb(dirPinPort, io_inpb(dirPinPort)&~(1<<slot));
}
void setPwm(unsigned int slot, unsigned int _duty, unsigned int _period){
  io_DisableINT();
  unsafeSetPwm(slot, _duty, _period);
  io_RestoreINT();
}
void unsafeSetPwm(unsigned int slot, unsigned int _duty, unsigned int _period){
  unsafeSetPwm(slot/3, slot%3, _duty, _period);
}
void unsafeSetDirectionAll(unsigned int val){
  io_outpb(dirPinPort, (io_inpb(dirPinPort)&(~(0x3fL)))|val);
}
void safeSetDirectionAll(unsigned int val){
  io_DisableINT();
  io_outpb(dirPinPort, (io_inpb(dirPinPort)&(~(0x3fL)))|val);
  io_RestoreINT();
}
byte unsafeReadLimitSwitchAll(){
  return io_inpb(GPIODATABASE + 20);
}
byte readLimitSwitchAll(){
  byte ls;
  io_DisableINT();
  ls = io_inpb(GPIODATABASE + 20);
  io_RestoreINT();
  return ls;
}
void digitalWriteCustomPin(unsigned int pin, bool stat){
  io_DisableINT();
  unsafeDigitalWriteCustomPin(pin, stat);
  io_RestoreINT();
}
bool digitalReadCustomPin(unsigned int pin){
  bool temp;
  io_DisableINT();
  temp = unsafeDigitalReadCustomPin(pin);
  io_RestoreINT();
  return temp;
}
void pinModeCustomPin(unsigned int pin, byte type){
  io_DisableINT();
  unsafePinModeCustomPin(pin, type);
  io_RestoreINT();
}
#define TRI_STATE     (0x00)
#define PULL_UP       (0x01)
#define PULL_DOWN     (0x02)

#define INPUT          (0x00)
#define OUTPUT         (0x01)
#define INPUT_PULLUP   (0x02)
#define INPUT_PULLDOWN (0x03)

void unsafeDigitalWriteCustomPin(unsigned int pin, bool stat){
  if(stat)//if HIGH
    io_outpb(GPIODATABASE + 4*(pin/8), io_inpb(GPIODATABASE + 4*(pin/8)) | 1<<(pin%8));
  else//if LOW
    io_outpb(GPIODATABASE + 4*(pin/8), io_inpb(GPIODATABASE + 4*(pin/8)) & ~(1<<(pin%8)));
}
bool unsafeDigitalReadCustomPin(unsigned int pin){
  if(io_inpb(GPIODATABASE + 4*(pin/8))&(1<<(pin%8))) return true;
  return false;
}

void unsafePinModeCustomPin(unsigned int pin, byte type){
  if(type == INPUT){
    io_outpb(CROSSBARBASE + 0x30 + pin, TRI_STATE);
    io_outpb(GPIODIRBASE + 4*(pin/8), io_inpb(GPIODIRBASE + 4*(pin/8))&~(1<<(pin%8)));   
  }else if(type == INPUT_PULLDOWN){
    io_outpb(CROSSBARBASE + 0x30 + pin, PULL_DOWN);
    io_outpb(GPIODIRBASE + 4*(pin/8), io_inpb(GPIODIRBASE + 4*(pin/8))&~(1<<(pin%8)));   
  }else if(type == INPUT_PULLUP){
    io_outpb(CROSSBARBASE + 0x30 + pin, PULL_UP);
    io_outpb(GPIODIRBASE + 4*(pin/8), io_inpb(GPIODIRBASE + 4*(pin/8))&~(1<<(pin%8)));   
  }else 
    io_outpb(GPIODIRBASE + 4*(pin/8), io_inpb(GPIODIRBASE + 4*(pin/8))|(1<<(pin%8)));   
}
static void (*isrPWM0)();
static void (*isrPWM1)();
static void (*isrPWM2)();
static void (*isrPWM3)();
static void (*isrPWM4)();
static void (*isrPWM5)();
static char* isrname_PWM_0 = "PWM 0 ISR";
static char* isrname_PWM_1 = "PWM 1 ISR";
static char* isrname_PWM_2 = "PWM 2 ISR";
static char* isrname_PWM_3 = "PWM 3 ISR";
static char* isrname_PWM_4 = "PWM 4 ISR";
static char* isrname_PWM_5 = "PWM 5 ISR";
void attachPwmInterrupt(void (*isr)()){
  disable_MCINT(0, 0);
  clear_INTSTATUS(0, 0);
  io_DisableINT();
  isrPWM0 = NULL;
  io_RestoreINT();
  if(irq_InstallISR(GetMCIRQ(), pwm_isr_handler0, isrname_PWM_0) == false){
    printf("irq_install fail\n"); 
    return;
  }
  enable_MCINT(0, 0, SC_END_INT);
  io_DisableINT();
  isrPWM0 = isr;//register real function to ISR function
  io_RestoreINT();
}
void attachPwmInterrupt(void (*isr)(), unsigned int slot){
  if(slot == 0){
    disable_MCINT(0, 0);
    clear_INTSTATUS(0, 0);
    io_DisableINT();
    isrPWM0 = NULL;
    io_RestoreINT();
    if(irq_InstallISR(GetMCIRQ(), pwm_isr_handler0, isrname_PWM_0) == false){
      printf("irq_install fail\n"); 
      return;
    }
    enable_MCINT(0, 0, SC_END_INT);
    io_DisableINT();
    isrPWM0 = isr;//register real function to ISR function
    io_RestoreINT();
  }else if(slot == 1){
    disable_MCINT(0, 1);
    clear_INTSTATUS(0, 1);
    io_DisableINT();
    isrPWM1 = NULL;
    io_RestoreINT();
    if(irq_InstallISR(GetMCIRQ(), pwm_isr_handler1, isrname_PWM_1) == false){
      printf("irq_install fail\n"); 
      return;
    }
    enable_MCINT(0, 1, SC_END_INT);
    io_DisableINT();
    isrPWM1 = isr;//register real function to ISR function
    io_RestoreINT();
  }else if(slot == 2){
    disable_MCINT(0, 2);
    clear_INTSTATUS(0, 2);
    io_DisableINT();
    isrPWM2 = NULL;
    io_RestoreINT();
    if(irq_InstallISR(GetMCIRQ(), pwm_isr_handler2, isrname_PWM_2) == false){
      printf("irq_install fail\n"); 
      return;
    }
    enable_MCINT(0, 2, SC_END_INT);
    io_DisableINT();
    isrPWM2 = isr;//register real function to ISR function
    io_RestoreINT();
  }else if(slot == 3){
    disable_MCINT(1, 0);
    clear_INTSTATUS(1, 0);
    io_DisableINT();
    isrPWM3 = NULL;
    io_RestoreINT();
    if(irq_InstallISR(GetMCIRQ(), pwm_isr_handler3, isrname_PWM_3) == false){
      printf("irq_install fail\n"); 
      return;
    }
    enable_MCINT(1, 0, SC_END_INT);
    io_DisableINT();
    isrPWM3 = isr;//register real function to ISR function
    io_RestoreINT();
  }else if(slot == 4){
    disable_MCINT(1, 1);
    clear_INTSTATUS(1, 1);
    io_DisableINT();
    isrPWM4 = NULL;
    io_RestoreINT();
    if(irq_InstallISR(GetMCIRQ(), pwm_isr_handler4, isrname_PWM_4) == false){
      printf("irq_install fail\n"); 
      return;
    }
    enable_MCINT(1, 1, SC_END_INT);
    io_DisableINT();
    isrPWM4 = isr;//register real function to ISR function
    io_RestoreINT();
  }else if(slot == 5){
    disable_MCINT(1, 2);
    clear_INTSTATUS(1, 2);
    io_DisableINT();
    isrPWM5 = NULL;
    io_RestoreINT();
    if(irq_InstallISR(GetMCIRQ(), pwm_isr_handler5, isrname_PWM_5) == false){
      printf("irq_install fail\n"); 
      return;
    }
    enable_MCINT(1, 2, SC_END_INT);
    io_DisableINT();
    isrPWM5 = isr;//register real function to ISR function
    io_RestoreINT();
  }else{
    io_RestoreINT();
    return;
  }
  
}
static int pwm_isr_handler0(int irq, void* data){
  if((mc_inp(0, 0x04) & (SC_END_INT << 0)) == 0) return 0;
  mc_outp(0, 0x04, (mc_inp(0, 0x04) & ~(0xffL<<0)) | (SC_END_INT << 0));//clear INT status bit
  if(isrPWM0 != NULL) isrPWM0();
  return 1;
}
static int pwm_isr_handler1(int irq, void* data){
  if((mc_inp(0, 0x04) & (SC_END_INT << 8)) == 0) return 0;
  mc_outp(0, 0x04, (mc_inp(0, 0x04) & ~(0xffL<<8)) | (SC_END_INT << 8));//clear INT status bit
  if(isrPWM1 != NULL) isrPWM1();
  return 1;
}
static int pwm_isr_handler2(int irq, void* data){
  if((mc_inp(0, 0x04) & (SC_END_INT << 16)) == 0) return 0;
  mc_outp(0, 0x04, (mc_inp(0, 0x04) & ~(0xffL<<16)) | (SC_END_INT << 16));//clear INT status bit
  if(isrPWM2 != NULL) isrPWM2();
  return 1;
}
static int pwm_isr_handler3(int irq, void* data){
  if((mc_inp(1, 0x04) & (SC_END_INT << 0)) == 0) return 0;
  mc_outp(1, 0x04, (mc_inp(1, 0x04) & ~(0xffL<<0)) | (SC_END_INT << 0));//clear INT status bit
  if(isrPWM3 != NULL) isrPWM3();
  return 1;
}
static int pwm_isr_handler4(int irq, void* data){
  if((mc_inp(1, 0x04) & (SC_END_INT << 8)) == 0) return 0;
  mc_outp(1, 0x04, (mc_inp(1, 0x04) & ~(0xffL<<8)) | (SC_END_INT << 8));//clear INT status bit
  if(isrPWM4 != NULL) isrPWM4();
  return 1;
}
static int pwm_isr_handler5(int irq, void* data){
  if((mc_inp(1, 0x04) & (SC_END_INT << 16)) == 0) return 0;
  mc_outp(1, 0x04, (mc_inp(1, 0x04) & ~(0xffL<<16)) | (SC_END_INT << 16));//clear INT status bit
  if(isrPWM5 != NULL) isrPWM5();
  return 1;
}
static int mcint_offset[3] = {0, 8, 16};
static void disable_MCINT(unsigned int mc, unsigned int md) {
    mc_outp(mc, 0x00, mc_inp(mc, 0x00) & ~(0xffL << mcint_offset[md]));  // disable mc interrupt
    mc_outp(MC_GENERAL, 0x38, mc_inp(MC_GENERAL, 0x38) | (1L << mc));
}
static void enable_MCINT(unsigned int mc, unsigned int md, unsigned long used_int) {
	mc_outp(MC_GENERAL, 0x38, mc_inp(MC_GENERAL, 0x38) & ~(1L << mc));
	mc_outp(mc, 0x00, (mc_inp(mc, 0x00) & ~(0xffL<<mcint_offset[md])) | (used_int << mcint_offset[md]));
}
static void clear_INTSTATUS(unsigned int mc, unsigned int md) {
    mc_outp(mc, 0x04, 0xffL << mcint_offset[md]); //for EX
}
//Below this is handler pattern, not used here
/*static int mc = 0;
static int md = 0;
static int mcint_offset[3] = {0, 8, 16};
static int timer1_isr_handler(int irq, void* data) {
    if((mc_inp(mc, 0x04) & (SC_END_INT << mcint_offset[md])) == 0) return 0;
    
	mc_outp(mc, 0x04, (SC_END_INT << mcint_offset[md]));
    
    if(PwmIsr.isrPWM != NULL) PwmIsr.isrPWM();
     
    return 1;
}*/


//Encoder
Encoder Enc[6] = {Encoder(0), Encoder(1), Encoder(2), Encoder(3), Encoder(4), Encoder(5)};
Encoder::Encoder(unsigned long _number){
  number = _number;
  if(number < 4){
    mdn = 1; //map to sensor B
    mcn = number;
  }else{
    mdn = 0; //map to sensor A
    mcn = 2+(number%4);
  }
}
void Encoder::begin(unsigned long _mode){
  mode = _mode;
  if(mode == MODE_AB_PHASE_x2) _pabInit(2);
  else if(mode == MODE_STEP_DIR) _pdirInit(1);
  else if(mode == MODE_STEP_DIR_x2) _pdirInit(2);
  //open_mcm_crossbar(mcn);//already open in HardwareInitialize()
  encoderEnable(mcn, mdn);
}
void Encoder::begin(){
  io_DisableINT();
  mode = MODE_AB_PHASE_x2;
  _pabInit(2);
  encoderEnable(mcn, mdn);
  io_RestoreINT();
}
static void encoderEnable(unsigned long mc, unsigned long md){
  if(md == 1)
    mc_outp(MC_GENERAL, MCG_ENABLEREG2, mc_inp(MC_GENERAL, MCG_ENABLEREG2) | (1L << (mc + 4)));//SIF module B
  else
    mc_outp(MC_GENERAL, MCG_ENABLEREG1, mc_inp(MC_GENERAL, MCG_ENABLEREG1) | (1L << (3*mc)));//SIF module A
}
static void encoderDisable(unsigned long mc, unsigned long md){
  if(md == 1)
    mc_outp(MC_GENERAL, MCG_ENABLEREG2, mc_inp(MC_GENERAL, MCG_ENABLEREG2) & ~(1L << (mc + 4)));//SIF module B
  else
    mc_outp(MC_GENERAL, MCG_ENABLEREG1, mc_inp(MC_GENERAL, MCG_ENABLEREG1) & ~(1L << (3*mc)));//SIF module A
}
unsigned long Encoder::read(){
  if(mode == MODE_AB_PHASE_x2 || mode == MODE_STEP_DIR){
    return mcenc_ReadPulCnt(mcn, mdn);
  }else return 0;
}
void Encoder::write(unsigned long cnt){
  if(mode == MODE_AB_PHASE_x2 || mode == MODE_STEP_DIR){
    encoderDisable(mcn, mdn);
    mcenc_SetPulCnt(mcn, mdn, cnt);
    encoderEnable(mcn, mdn);
  }else return;
}
unsigned long Encoder::readCapFIFO(){
  return mc_inp(mcn, MCSIF_modOffset[mdn] + MCSIF_ENC_CAPFIFOREG);
}
void Encoder::_pabInit(int samplerate){
  //_filterAndSampleWindowInit(mcn, mdn);
  _filterAndSampleWindowInitWithWholeFilter(mcn, mdn, 10L);
  mcsif_SetMode(mcn, mdn, MCSIF_ENC_PAB);
  _defaultEncoderSetting(mcn, mdn);
  io_DisableINT();
  if(samplerate == 1){
    mcenc_SetCntMode(mcn, mdn, MCENC_PAB_DIR0_INCA + MCENC_PAB_DIR1_DECA);
  }else{
    mcenc_SetCntMode(mcn, mdn, MCENC_PAB_DIR0_INCAB + MCENC_PAB_DIR1_DECAB);
  }
  io_RestoreINT();
}
void Encoder::_pdirInit(int samplerate){
  //_filterAndSampleWindowInitWithWholeFilter(mcn, mdn, 50L);
  //_filterAndSampleWindowInitWithSeperatedFilter(mcn, mdn, 50L);
  _filterAndSampleWindowInitWithSeperatedFilter(mcn, mdn, 50L);
  mcsif_SetMode(mcn, mdn, MCSIF_ENC_PDIR);
  _defaultEncoderSetting(mcn, mdn);
  io_DisableINT();
  if(samplerate == 1){
    mcenc_SetCntMode(mcn, mdn, MCENC_PDIR_DIR0_INC0TO1 + MCENC_PDIR_DIR1_DEC0TO1);
  }else{
    mcenc_SetCntMode(mcn, mdn, MCENC_PDIR_DIR0_INCBOTH + MCENC_PDIR_DIR1_DECBOTH);
  }
  io_RestoreINT();
}
static void _defaultEncoderSetting(int mc, int md) {
    mcenc_SetIdxCond(mc, md, MCENC_PDIR_IDXCOND_DISABLE, 0L);
    mcenc_SetCapMode(mc, md, MCENC_CAP_PCNT_ENABLE_CLEAR + MCENC_CAP_EXTRIG_DISABLE + MCENC_CAP_IDXCOND_DISABLE);
    mcenc_SetCapInterval(mc, md, 1L);
    mcenc_SetCntMin(mc, md, 0L);
    mcenc_SetCntMax(mc, md, 0xffffffffL);
    mcenc_SetResetMode(mc, md, MCENC_RESET_INC_CNTMIN + MCENC_RESET_DEC_CNTMAX);
    mcenc_SetCntIdx(mc, md, 0L);
    mcenc_SetTrigResetMode(mc, md, MCENC_TRIGRESET_IDXCOND_0TO1);
    mcenc_SetPulCnt(mc, md, 0L);
}
static void _filterAndSampleWindowInit(int mc, int md) {
	mcsif_SetInputFilter(mc, md, 0L);
  	mcsif_SetSWDeadband(mc, md, 0L);
  	mcsif_SetSWPOL(mc, md, MCSIF_SWPOL_REMAIN);
  	mcsif_SetSamplWin(mc, md, MCSIF_SWSTART_DISABLE + MCSIF_SWEND_NOW);
  	mcsif_SetSamplWin(mc, md, MCSIF_SWSTART_NOW + MCSIF_SWEND_DISABLE);
}
static void _filterAndSampleWindowInitWithWholeFilter(int mc, int md, unsigned long filtime) {
        mc_outp(mc, MCSIF_modOffset[md] + MCSIF_FILREG, (filtime & 0x0000ffffL) + 0x80000000L);
  	mcsif_SetSWDeadband(mc, md, 0L);
  	mcsif_SetSWPOL(mc, md, MCSIF_SWPOL_REMAIN);
  	mcsif_SetSamplWin(mc, md, MCSIF_SWSTART_DISABLE + MCSIF_SWEND_NOW);
  	mcsif_SetSamplWin(mc, md, MCSIF_SWSTART_NOW + MCSIF_SWEND_DISABLE);
}
static void _filterAndSampleWindowInitWithSeperatedFilter(int mc, int md, unsigned long filtime) {
        mc_outp(mc, MCSIF_modOffset[md] + MCSIF_FILREG, (filtime & 0x0000ffffL));
  	mcsif_SetSWDeadband(mc, md, 0L);
  	mcsif_SetSWPOL(mc, md, MCSIF_SWPOL_REMAIN);
  	mcsif_SetSamplWin(mc, md, MCSIF_SWSTART_DISABLE + MCSIF_SWEND_NOW);
  	mcsif_SetSamplWin(mc, md, MCSIF_SWSTART_NOW + MCSIF_SWEND_DISABLE);
}




