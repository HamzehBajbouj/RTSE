/*
 *   ROBOSAMPLE.C -- A sample/template for RoboKar program with uCOS-II
 *   Written by: Rosbi Mamat 6/5/2014
 *   Updated : 1/5/2023 Modified to show proximity & light sensor usage
 */

#include "..\inc\kernel.h"                  /* Always include these to use uCOS-II      */
#include "..\inc\hal_robo.h"                /*   and RoboKar HAL                        */

#define TASK_STK_SZ            128          /* Size of each task's stacks (# of bytes)  */
#define TASK_START_PRIO          2          /* Highest priority                         */
#define TASK_CHKCOLLIDE_PRIO     5
#define TASK_CTRLMOTOR_PRIO      3
#define TASK_NAVIG_PRIO          1         /* Lowest priority                          */
#define TASK_LINE_PRIO          4


#define TARGET_LINE_POSITION 1000       
#define MAX_INTEGRAL_TERM 50000  


OS_STK TaskStartStk[TASK_STK_SZ];           /* TaskStartTask stack                      */
OS_STK ChkCollideStk[TASK_STK_SZ];          /* Task StopOnCollide stack                 */
OS_STK CtrlmotorStk[TASK_STK_SZ];           /* Task CtrlMotors stack                    */
OS_STK NavigStk[TASK_STK_SZ];               /* Task NavigRobot stack                    */
OS_STK LineStk[TASK_STK_SZ]; 
/* ------ Global shared variable -------*/
/* Ideally, this should be protected by a semaphore etc */
struct robostate
{
    int rspeed;                             /* right motor speed  (-100 -- +100)        */
    int lspeed;                             /* leftt motor speed  (-100 -- +100)        */
    char obstacle;                          /* obstacle? 1 = yes, 0 = no                */
    double prevError;
    double integral;
} myrobot;

/*------High pririority task----------*/
void CheckCollision (void *data)
{
    for(;;)
    {
        if ( (robo_proxSensor() == 1) )             /* obstacle?                         */
            myrobot.obstacle = 1;                   /* signal obstacle present           */
        else
            myrobot.obstacle = 0;                   /* signal no obstacle                */

		OSTimeDlyHMSM(0, 0, 0, 100);                /* Task period ~ 100 ms              */
    }
}

/* Control robot Motors TASK */
void CntrlMotors (void *data)
{
    int speed_r, speed_l;

    for(;;)
    {
        speed_r = myrobot.rspeed;
        speed_l = myrobot.lspeed;
        robo_motorSpeed(speed_l, speed_r);
        OSTimeDlyHMSM(0, 0, 0, 10);                /* Task period ~ 250 ms              */
    }
}


int normalizeSensorReading (int sensorReading)
{
//    switch statement, if 0 then 0, if 7 then then 150, if 2 then 50
    switch (sensorReading)
    {
        case 0:
            return 2000;
        // turn right:
        case 1: 
            return 3000;
        case 3: 
            return 1500;
        // center: 
        case 7:
            return 1000;
        case 2: 
            return 1000; 
        // turn left
        case 4: 
            return 0;
        case 6: 
            return 500;
        default: 
            return 0;
        
    }

}

/* --- Task for navigating robot ----
 * Write you own navigation task here
 */

void Navig (void *data)
{

     // PID controller variables
    double Kp = 0.1;     // Proportional gain
    double Ki = 0.0000;     // Integral gain
    double Kd = 0;     // Derivative gain



    for (;;)
    {

        // Line sensor measurement
        int rawReading = robo_lineSensor();
        
        int lineSensorReading = normalizeSensorReading(rawReading);
        // cprintf("line sensor reading: %d \r\n", lineSensorReading);


          // Calculate error
        double error = TARGET_LINE_POSITION - lineSensorReading;

         // Update integral term
        myrobot.integral += error;

        // Limit the integral term to prevent excessive accumulation
        if (myrobot.integral > MAX_INTEGRAL_TERM)
            myrobot.integral = MAX_INTEGRAL_TERM;
        else if (myrobot.integral < -MAX_INTEGRAL_TERM)
            myrobot.integral = -MAX_INTEGRAL_TERM;

        // Calculate control signal
        double controlSignal = (Kp * error) + (Ki * myrobot.integral) + (Kd * (error - myrobot.prevError));

        // Update previous error
        myrobot.prevError = error;

        // Update motor speeds based on control signal
        myrobot.rspeed = (MEDIUM_SPEED + (int)controlSignal);
        myrobot.lspeed = (MEDIUM_SPEED - (int)controlSignal);

        if (myrobot.rspeed > 100)
            myrobot.rspeed = 100;
        else if (myrobot.rspeed < -100)
            myrobot.rspeed = -100;

        if (myrobot.lspeed > 100)
            myrobot.lspeed = 100;
        else if (myrobot.lspeed < -100)
            myrobot.lspeed = -100;


        // cprintf("rspeed: %d, lspeed: %d \r\n", myrobot.rspeed, myrobot.lspeed);
       
        OSTimeDlyHMSM(0, 0, 0, 10);                /* Task period ~ 500 ms                  */
    }
}

void TestLineSensor (void *data)
{
    for (;;)
    {

	//cputchar(robo_lineSensor());

//	cputchar(	robo_lightSensor());
	//  cprintf("%d",robo_lineSensor());
//	 cprintf("fdsd");
    //   OSTimeDlyHMSM(0, 0, 0, 50);               
    }
}


/*------Navigation controllers utils----------*/
void turnRight(void *data){
    myrobot.rspeed   = -MEDIUM_SPEED;       
    myrobot.lspeed   = MEDIUM_SPEED;
}

void turnLeft(void *data){
    myrobot.rspeed   = MEDIUM_SPEED;       
    myrobot.lspeed   = -MEDIUM_SPEED;
}

void goStraight(void *data){
    myrobot.rspeed   = HIGH_SPEED;       
    myrobot.lspeed   = HIGH_SPEED;
}

void goBackword(void *data){
    myrobot.rspeed   = -HIGH_SPEED;       
    myrobot.lspeed   = -HIGH_SPEED;
}


/*------Highest pririority task----------*/
/* Create all other tasks here           */
void TaskStart( void *data )
{
    OS_ticks_init();                                        /* enable RTOS timer tick        */

    OSTaskCreate(CheckCollision,                            /* Task function                 */
                (void *)0,                                  /* nothing passed to task        */
                (void *)&ChkCollideStk[TASK_STK_SZ - 1],    /* stack allocated to task       */
                TASK_CHKCOLLIDE_PRIO);                      /* priority of task              */

    OSTaskCreate(CntrlMotors,                               /* Task function                 */
                (void *)0,                                  /* nothing passed to task        */
                (void *)&CtrlmotorStk[TASK_STK_SZ - 1],     /* stack allocated to task       */
                TASK_CTRLMOTOR_PRIO);                       /* priority of task              */

    OSTaskCreate(Navig,                                     /* Task function                 */
                (void *)0,                                  /* nothing passed to task        */
                (void *)&NavigStk[TASK_STK_SZ - 1],         /* stack allocated to task       */
                TASK_NAVIG_PRIO);                           /* priority of task              */

 OSTaskCreate(TestLineSensor,                                     /* Task function                 */
                (void *)0,                                  /* nothing passed to task        */
                (void *)&LineStk[TASK_STK_SZ - 1],         /* stack allocated to task       */
                TASK_LINE_PRIO); 
    while(1)
    {
        OSTimeDlyHMSM(0, 0, 5, 0);                          /* Task period ~ 5 secs          */
        // robo_LED_toggle();                                  /* Show that we are alive        */
	//	cprintf("%d",robo_proxSensor());
	//	cprintf("ddsd");
    }

}

int main( void )
{
    robo_Setup();                                          /* initialize HAL for RoboKar     */
    OSInit();                                              /* initialize UCOS-II kernel      */

    robo_motorSpeed(STOP_SPEED, STOP_SPEED);               /* Stop the robot                 */
    myrobot.rspeed   = STOP_SPEED;                         /* Initialize myrobot states      */
    myrobot.lspeed   = STOP_SPEED;
    myrobot.obstacle = 0;    
                                  /*  No collisioin                 */

    myrobot.prevError = 0.0;
    myrobot.integral = 0.0;
    OSTaskCreate(TaskStart,                                /* create TaskStart Task          */
                (void *)0,
                (void *)&TaskStartStk[TASK_STK_SZ - 1],
                TASK_START_PRIO);
	robo_Honk(); robo_wait4goPress();                      /* Wait for to GO                 */
    OSStart();                                             /* Start multitasking             */
    while (1);                                             /* die here                       */
}

