#ifndef MCMServiceConfig_h
#define MCMServiceConfig_h
//this header use to set how service works.
#define SERVO_MAIN_INTERRUPT 0 //this determine Motor PWM number to be used as interrupt based.
#define SERVO_RATE_SCALER 2 // use to set servo rate to pwm frequency ratio. For 20kHz pwm with scaler = 2, servo rate = 10kHz

//DIR_STEP Pulse Generator
//#define USE_AUX_DIR_STEP_0 //this feature is using dir_6 and dir_7 as DIR_STEP pulse generation, comment if not used
#define AUX_DIR_STEP_0_DELAY 1 //in us
#define AUX_DIR_STEP_0_MAX_PULSE_PER_LOOP 1//maximum pulse generate per 1 servo loop.


//Below this are for reading encoder in service. Comment them if reading is not desired.
//consume 1 us each
#define ENCODER_0_USED 
#define ENCODER_1_USED
#define ENCODER_2_USED
#define ENCODER_3_USED
#define ENCODER_4_USED
#define ENCODER_5_USED


//These set mode of encoder, There are...
//MODE_AB_PHASE_x2 for X4 Quadrature decoder
//MODE_STEP_DIR for counting STEP-DIR

#define ENCODER_0_MODE MODE_AB_PHASE_x2
#define ENCODER_1_MODE MODE_AB_PHASE_x2
#define ENCODER_2_MODE MODE_AB_PHASE_x2
#define ENCODER_3_MODE MODE_STEP_DIR
#define ENCODER_4_MODE MODE_STEP_DIR
#define ENCODER_5_MODE MODE_STEP_DIR

//Below this are motor power updating. Comment them if not used
//Enable a motor will consume approximately 2.5us per motor per servo loop
#define MOTOR_0_USED
#define MOTOR_1_USED
#define MOTOR_2_USED
#define MOTOR_3_USED
#define MOTOR_4_USED
#define MOTOR_5_USED

//set motor driver type, 0 for sign-magnitude, 1 for locked anti-phase
#define MOTOR_0_TYPE 0
#define MOTOR_1_TYPE 0
#define MOTOR_2_TYPE 0
#define MOTOR_3_TYPE 0
#define MOTOR_4_TYPE 0
#define MOTOR_5_TYPE 0

//clock for PWM Period, 1 clock is 10 ns
//for 50us, 20kHz, clock is 5000
//Too high interrupt rate will result in system freeze, need to be flashed by SD card
#define MOTOR_0_PWM_CLOCK 5000
#define MOTOR_1_PWM_CLOCK 5000
#define MOTOR_2_PWM_CLOCK 5000
#define MOTOR_3_PWM_CLOCK 5000
#define MOTOR_4_PWM_CLOCK 5000
#define MOTOR_5_PWM_CLOCK 5000

//analogRead
//consume 1.25us each
//#define ADC_0_USED
//#define ADC_1_USED
//#define ADC_2_USED
//#define ADC_3_USED
//#define ADC_4_USED
//#define ADC_5_USED
//#define ADC_6_USED
//#define ADC_7_USED

#define UART_MAX_TRANSMIT_PER_LOOP 16 //max 32
#define UART_MAX_RECEIVE_PER_LOOP 16 //max 32

#define READ_LIMIT_SWITCH //comment if no need to read limit switch

//below this is calculation part, do not manipulate

#if SERVO_MAIN_INTERRUPT == 0
  #define SERVO_RATE_REAL 100000000/(SERVO_RATE_SCALER*MOTOR_0_PWM_CLOCK)
  #define SERVO_SAMPLING_TIME_REAL (SERVO_RATE_SCALER*MOTOR_0_PWM_CLOCK)/100000000.0
  #define SERVO_SAMPLING_TIME_US (SERVO_RATE_SCALER*MOTOR_0_PWM_CLOCK)/100
#endif
#if SERVO_MAIN_INTERRUPT == 1
  #define SERVO_RATE_REAL 100000000/(SERVO_RATE_SCALER*MOTOR_1_PWM_CLOCK)
  #define SERVO_SAMPLING_TIME_REAL ((SERVO_RATE_SCALER*MOTOR_1_PWM_CLOCK)/100000000.0)
  #define SERVO_SAMPLING_TIME_US (SERVO_RATE_SCALER*MOTOR_1_PWM_CLOCK)/100
#endif
#if SERVO_MAIN_INTERRUPT == 2
  #define SERVO_RATE_REAL 100000000/(SERVO_RATE_SCALER*MOTOR_2_PWM_CLOCK)
  #define SERVO_SAMPLING_TIME_REAL ((SERVO_RATE_SCALER*MOTOR_2_PWM_CLOCK)/100000000.0)
  #define SERVO_SAMPLING_TIME_US (SERVO_RATE_SCALER*MOTOR_2_PWM_CLOCK)/100
#endif
#if SERVO_MAIN_INTERRUPT == 3
  #define SERVO_RATE_REAL 100000000/(SERVO_RATE_SCALER*MOTOR_3_PWM_CLOCK)
  #define SERVO_SAMPLING_TIME_REAL ((SERVO_RATE_SCALER*MOTOR_3_PWM_CLOCK)/100000000.0)
  #define SERVO_SAMPLING_TIME_US (SERVO_RATE_SCALER*MOTOR_3_PWM_CLOCK)/100
#endif
#if SERVO_MAIN_INTERRUPT == 4
  #define SERVO_RATE_REAL 100000000/(SERVO_RATE_SCALER*MOTOR_4_PWM_CLOCK)
  #define SERVO_SAMPLING_TIME_REAL ((SERVO_RATE_SCALER*MOTOR_4_PWM_CLOCK)/100000000.0)
  #define SERVO_SAMPLING_TIME_US (SERVO_RATE_SCALER*MOTOR_4_PWM_CLOCK)/100
#endif
#if SERVO_MAIN_INTERRUPT == 5
  #define SERVO_RATE_REAL 100000000/(SERVO_RATE_SCALER*MOTOR_5_PWM_CLOCK)
  #define SERVO_SAMPLING_TIME_REAL ((SERVO_RATE_SCALER*MOTOR_5_PWM_CLOCK)/100000000.0)
  #define SERVO_SAMPLING_TIME_US (SERVO_RATE_SCALER*MOTOR_5_PWM_CLOCK)/100
#endif


#endif
