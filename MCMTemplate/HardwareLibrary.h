#ifndef HardwareLibrary_h
#define HardwareLibrary_h
#include <stdint.h>
#include "Stream.h"
#include "com.h"
#include "HardwareSerial.h"
#define byte uint8_t
#define POLLING_SERIAL_BUFFER_SIZE 4096
#define POLLING_SERIAL_HARDWARE_BUFFER_SIZE 32
class PollingSerial : public Stream{
  private:
    int number;
    int port; //1, 2, 3
    unsigned long baudrate;
    unsigned char format;
    unsigned long rxtimeout;
    unsigned long txtimeout;
    bool peek_stored;
    int peek_val;
    
  public:
    PollingSerial(int _number, unsigned long com_buadrate, unsigned char com_format, unsigned long com_rxtimeout, unsigned long com_txtimeout);
    void begin(unsigned long);
    void begin(unsigned long, int);
    void begin(unsigned long, uint8_t);
    void begin(unsigned long , uint8_t, int);
    void end();
    virtual int available(void);
    virtual int peek(void);
    virtual int read(void);
    virtual void flush(void);
    virtual size_t write(uint8_t);
    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n) { return write((uint8_t)n); }
    inline size_t write(unsigned int n) { return write((uint8_t)n); }
    inline size_t write(int n) { return write((uint8_t)n); }
    using Print::write; // pull in write(str) and write(buf, size) from Print
    unsigned short uartConfigBaseAddress = 0;
    unsigned short uartBaseAddress = 0;
    void poll(unsigned long maxTransmitBytes, unsigned long maxReceiveBytes);
    byte sendingBuffer[POLLING_SERIAL_BUFFER_SIZE];
    byte receivingBuffer[POLLING_SERIAL_BUFFER_SIZE];
    volatile unsigned long sendingBufferPosition = 0;
    volatile unsigned long receivingBufferPosition = 0;
    volatile unsigned long bufferNotSent = 0;
    volatile unsigned long bufferNotRead = 0;
    volatile unsigned long sendingBufferPositionForPolling = 0;
    volatile unsigned long receivingBufferPositionForPolling = 0;
    volatile bool initialized = false;
};
extern PollingSerial pSerial1;
extern PollingSerial pSerial2;
extern PollingSerial pSerial3;

//setup function
void HardwareInitialize();//initialize hardware, must be called in setup()
void PwmInitialize(unsigned int slot, unsigned int _period);//initialize PWM module, slot = 0-5, period unit is clock (10 ns)
//usable function
//function with "unsafe" must be called while interrupt is disabled
//do not use function without "unsafe" in realTime service, since there is io_RestoreINT() at the end of function
void setPwm(unsigned int slot, unsigned int _duty, unsigned int _period);
void unsafeSetPwm(unsigned int slot, unsigned int _duty, unsigned int _period);
void setDirection(unsigned int slot, bool _dir);
void unsafeSetDirection(unsigned int slot, bool _dir);
byte readLimitSwitchAll();
byte unsafeReadLimitSwitchAll();
void digitalWriteCustomPin(unsigned int pin, bool stat);
void unsafeDigitalWriteCustomPin(unsigned int pin, bool stat);
bool digitalReadCustomPin(unsigned int pin);
bool unsafeDigitalReadCustomPin(unsigned int pin);
void pinModeCustomPin(unsigned int pin, byte type);
void unsafePinModeCustomPin(unsigned int pin, byte type);

//define pin
#define PIN_DIR_0 32
#define PIN_DIR_1 33
#define PIN_DIR_2 34
#define PIN_DIR_3 35
#define PIN_DIR_4 36
#define PIN_DIR_5 37
#define PIN_DIR_6 38
#define PIN_DIR_7 39

#define PIN_LM_0 40
#define PIN_LM_1 41
#define PIN_LM_2 42
#define PIN_LM_3 43
#define PIN_LM_4 44
#define PIN_LM_5 45
#define PIN_LM_6 46
#define PIN_LM_7 47


//below are functions used in MCMService.h
void PwmInitializeWithSampleCycle(unsigned int slot, unsigned int _period, unsigned int _sc);
void PwmSetSampleCycle(unsigned int slot, unsigned int _sc);
void safeSetPwm(unsigned int i, unsigned int j, unsigned int _duty, unsigned int _period);
void unsafeSetPwm(unsigned int i, unsigned int j, unsigned int _duty, unsigned int _period);//must be called only while io_DisableINT(); is called.
void safeSetDirectionAll(unsigned int val);
void unsafeSetDirectionAll(unsigned int val);//called while interrupt are disabled only.
void attachPwmInterrupt(void (*isr)(), unsigned int slot);//slot is from 0-5
void attachPwmInterrupt(void (*isr)());
static void disable_MCINT(unsigned int mc, unsigned int md);
static void enable_MCINT(unsigned int mc, unsigned int md, unsigned long used_int);
static void clear_INTSTATUS(unsigned int mc, unsigned int md);
static int pwm_isr_handler0(int irq, void* data);
static int pwm_isr_handler1(int irq, void* data);
static int pwm_isr_handler2(int irq, void* data);
static int pwm_isr_handler3(int irq, void* data);
static int pwm_isr_handler4(int irq, void* data);
static int pwm_isr_handler5(int irq, void* data);


// Encoder mode
#define MODE_NOSET       (0xFF)
#define MODE_STEP_DIR    (0)
#define MODE_CWCCW       (1)
#define MODE_AB_PHASE    (2)
#define MODE_CAPTURE     (3)
#define MODE_SSI         (4) // continue mode, no interrupt event
#define MODE_STEP_DIR_x2 (5) // another mode
#define MODE_CWCCW_x2    (6) // another mode
#define MODE_AB_PHASE_x2 (7) // another mode
class Encoder{
  public:
    Encoder(unsigned long _number);
    void begin(unsigned long _mode);
    void begin();//for automatic set to X4
    unsigned long read();
    void write(unsigned long);
    unsigned long readCapFIFO();
  private:
    unsigned long number = 0;
    unsigned long mcn = 0;
    unsigned long mdn = 0;
    unsigned long mode = 0;
    void _pabInit(int samplerate);
    void _pdirInit(int sampleRate);
};

static void _defaultEncoderSetting(int mc, int md);
static void _filterAndSampleWindowInit(int mc, int md);
static void _filterAndSampleWindowInitWithWholeFilter(int mc, int md, unsigned long filtime);
static void _filterAndSampleWindowInitWithSeperatedFilter(int mc, int md, unsigned long filtime);
static void encoderEnable(unsigned long mc, unsigned long md);
static void encoderDisable(unsigned long mc, unsigned long md);
extern Encoder Enc[6];





#endif
