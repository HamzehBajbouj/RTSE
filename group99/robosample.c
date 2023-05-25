/*
 *   ROBOSAMPLE.C -- A sample/template for RoboKar program with uCOS-II
 *   Written by: Rosbi Mamat 6/5/2014
 *   Updated : 1/5/2023 Modified to show proximity & light sensor usage
 */

#include "..\inc\kernel.h"                  /* Always include these to use uCOS-II      */
#include "..\inc\hal_robo.h"                /*   and RoboKar HAL                        */

#define TASK_STK_SZ            128          /* Size of each task's stacks (# of bytes)  */
#define TASK_START_PRIO          1          /* Highest priority                         */
#define TASK_CHKCOLLIDE_PRIO     2
#define TASK_CTRLMOTOR_PRIO      3
#define TASK_NAVIG_PRIO          4          /* Lowest priority                          */
#define TASK_LINE_PRIO          5    

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
        OSTimeDlyHMSM(0, 0, 0, 250);                /* Task period ~ 250 ms              */
    }
}

/* --- Task for navigating robot ----
 * Write you own navigation task here
 */

void Navig (void *data)
{
    for (;;)
    {
        if (myrobot.obstacle == 1)                  /* If blocked then reverse              */
        {
            myrobot.rspeed   = -LOW_SPEED;          /* REVERSE */
            myrobot.lspeed   = -LOW_SPEED;
        }
        else                                        /* obstacle is far away & no collision   */
        {
            myrobot.rspeed   = MEDIUM_SPEED;        /* move forward with medium speed        */
            myrobot.lspeed   = MEDIUM_SPEED;
        }

		if (robo_lightSensor() > 60)                /* it is too bright, I'm photophobia     */
		{
			myrobot.rspeed   = -LOW_SPEED;          /* turn right to avoid                   */
            myrobot.lspeed   =  LOW_SPEED;
		}
        OSTimeDlyHMSM(0, 0, 0, 500);                /* Task period ~ 500 ms                  */
    }
}

void TestLineSensor (void *data)
{
    for (;;)
    {

	 cputchar(robo_lineSensor());
      OSTimeDlyHMSM(0, 0, 0, 250);               
    }
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
        robo_LED_toggle();                                  /* Show that we are alive        */
    }

}

int main( void )
{
    robo_Setup();                                          /* initialize HAL for RoboKar     */
    OSInit();                                              /* initialize UCOS-II kernel      */

    robo_motorSpeed(STOP_SPEED, STOP_SPEED);               /* Stop the robot                 */
    myrobot.rspeed   = STOP_SPEED;                         /* Initialize myrobot states      */
    myrobot.lspeed   = STOP_SPEED;
    myrobot.obstacle = 0;                                  /*  No collisioin                 */

    OSTaskCreate(TaskStart,                                /* create TaskStart Task          */
                (void *)0,
                (void *)&TaskStartStk[TASK_STK_SZ - 1],
                TASK_START_PRIO);
	robo_Honk(); robo_wait4goPress();                      /* Wait for to GO                 */
    OSStart();                                             /* Start multitasking             */
    while (1);                                             /* die here                       */
}

