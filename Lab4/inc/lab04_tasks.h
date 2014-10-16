/* Rice ELEC424 lab04_tasks.h */
/* Simulated Quadrotor tasks */
/* Authors: Steven Arroyo and Lin Zhong */

/* 
 * Speed struct for motors
 */
typedef struct
{
    unsigned char m1;
    unsigned char m2;
    unsigned char m3;
    unsigned char m4;
} MotorSpeeds;  

/*
 * This function needs to be executed once every 10 ms.  Priority 1.
 */
unsigned char detectEmergency(void); 

/* 
 * This function needs to be executed once every 100 ms.  Priority 2.
 */
unsigned char refreshSensorData(void); 

/* 
 * This function needs to be executed once every second.    Priority 3.
 * This function takes many cycles to complete. As such, care should be 
 * taken not to block other higher priority functions
 */

unsigned char calculateOrientation(void); 

/* 
 * This function needs to be executed once every second.    Priority 4.
 * This function will return a struct with on-off settings for each motor.  Since the PWM's can
 * be configured in different ways the speed will be either 0 or 1 (off or on).
 */
unsigned char updatePid(MotorSpeeds* p_motorSpeedsPtr);

/*
 * Run this low priority function when time is available.    Priority 5.
 */
unsigned char logDebugInfo(void);

