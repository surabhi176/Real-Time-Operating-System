// RTOS Framework - Spring 2020
// J Losh

// Student Names: Surabhi Chythanya Kumar 
// TO DO: Add your name on this line.  Do not include your ID number in the file.

// Add xx_ prefix to all files in your project
// xx_rtos.c
// xx_tm4c123gh6pm_startup_ccs.c
// xx_other files (except uart0.x and wait.x)
// (xx is a unique number that will be issued in class)
// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 6 Pushbuttons and 5 LEDs, UART
// LEDs on these pins:
// Blue:   PF2 (on-board)
// Red:    PA2
// Orange: PA3
// Yellow: PA4
// Green:  PE0
// PBs on these pins
// PB0:    PC4
// PB1:    PC5
// PB2:    PC6
// PB3:    PC7
// PB4:    PD6
// PB5:    PD7
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
//#include "uart0.h"
#include "wait.h"

// REQUIRED: correct these bitbanding references for the off-board LEDs-done
#define BLUE_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define RED_LED    (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4)))
#define ORANGE_LED    (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4)))
#define YELLOW_LED    (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 0*4)))

#define PUSH_BUTTON0  (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 4*4)))
#define PUSH_BUTTON1  (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4)))
#define PUSH_BUTTON2  (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))
#define PUSH_BUTTON3  (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 7*4)))
#define PUSH_BUTTON4  (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 6*4)))
#define PUSH_BUTTON5  (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 7*4)))


#define BLUE_LED_MASK 4
#define RED_LED_MASK 4
#define ORANGE_LED_MASK 8
#define YELLOW_LED_MASK 16
#define GREEN_LED_MASK 1

#define PUSH_BUTTON0_MASK 16
#define PUSH_BUTTON1_MASK 32
#define PUSH_BUTTON2_MASK 64
#define PUSH_BUTTON3_MASK 128
#define PUSH_BUTTON4_MASK 64
#define PUSH_BUTTON5_MASK 128

#define keyPressed 0
#define keyReleased 1
#define flashReq 2
#define resource 3
//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
struct semaphore
{
    uint16_t count;
    uint16_t queueSize;
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
    char name[16];
    uint8_t currTask;
} semaphores[MAX_SEMAPHORES];
uint8_t semaphoreCount = 0;

//struct semaphore *keyPressed, *keyReleased, *flashReq, *resource;
//struct semaphore semName[MAX_SEMAPHORES];


// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define STATE_BLOCKED    4 // has run, but now blocked by semaphore

#define MAX_TASKS 12       // maximum number of valid tasks

#define MAX_CHARS 80
#define MAX_FIELDS 6
#define delay4Cycles() __asm(" NOP\n NOP\n NOP\n NOP")

extern void setPSP(uint32_t *sp);
extern void pushR4_11_PSP();
extern void *getPSP();

extern void popR4_11_PSP();
extern void pushR0_xPSR(uint32_t *pid);
extern void exc_ret();
extern void setPSP1(uint32_t *sp);
extern uint32_t getR0();
extern void setTMPL();
extern uint32_t get_MSP();
extern uint32_t getR2();
extern uint32_t getR3();
extern uint32_t getR12();
extern uint32_t getLR();
extern uint32_t getPC();
extern uint32_t getxPSR();
extern uint32_t getR1();
extern uint32_t get_PSP();

void putsUart0(char* );
uint8_t itoa(uint16_t , char [], uint8_t );

uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks

uint32_t stack[10][512] __attribute__((aligned (2048)));


// REQUIRED: add store and management for the memory used by the thread stacks
//           thread stacks must start on 1 kiB boundaries so mpu can work correctly

struct _tcb
{
    uint8_t state;                 // see STATE_ values above
    void *pid;                     // used to uniquely identify thread
    void *spInit;                  // location of original stack pointer
    void *sp;                      // location of stack pointer for thread
    int8_t priority;               // 0=highest to 15=lowest
    int8_t currentPriority;        // used for priority inheritance
    uint32_t ticks;                // ticks until sleep complete
    char name[16];                 // name of task used in ps command
    void *semaphore;               // pointer to the semaphore that is blocking the thread
    uint32_t sum[2];               //for ping-pong operation to calculate cpu percentage
    uint32_t flag;

} tcb[MAX_TASKS];

struct _shell1
{
    char str[MAX_CHARS+1];
    uint8_t position[MAX_FIELDS];
    uint8_t argCount;
    uint8_t min_args;
    char strout[MAX_CHARS];
    uint8_t mode;                      //to change schedular mode between priority and round robin
    uint32_t pid;
    bool preempt;                      //to turn on or off preemption
    bool pi;                           //to turn on or off priority inheritance
} shell1;

struct _timer                   //to calculate cpu percentage
{
    uint32_t t1,t2; //value of timer count
    uint32_t deltat;
    uint32_t task_percent;
    uint32_t sum;
    uint32_t task_point;
}time[MAX_TASKS];
struct _j
{
    uint32_t j;
}j1;

//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

// REQUIRED: initialize systick for 1ms system timer
void initRtos()
{
    uint8_t i;
    // no tasks running
    taskCount = 0;
    // clear out tcb records
    for (i = 0; i < MAX_TASKS; i++)
    {
        tcb[i].state = STATE_INVALID;
        tcb[i].pid = 0;
    }
}

// REQUIRED: Implement prioritization to 16 levels
int rtosScheduler()
{
    bool ok;
    static uint8_t task = 0xFF;
    uint16_t count=0,priority=0,taskPriority,i;

    {
        if(tcb[taskCurrent].state==1)
        {
            i=taskCurrent;
        }
        else
        {
            if(taskCurrent==8)
               {
                   i=0;
               }
            else
            i=taskCurrent+1;
        }
    }
            uint8_t taskState;

    //Priority schedular
     while(shell1.mode==1)
      {



    taskPriority=tcb[i].currentPriority;
         taskState=tcb[i].state;

         if(priority==taskPriority)
         {
             if(taskState==2 || taskState==1)
                 return i;
         }
         {
             i++;
             count++;
             if(i==9)
             {
                 i=0;
             }
             if(count==9)
             {
                 priority++;
                 if(priority==16)
                 {
                     priority=0;
                 }
                 count=0;
             }
         }

      }

     //Round Robin Schedular
     while(shell1.mode==0)
    {
    ok = false;
    while (!ok)
    {
        task++;
        if (task >= MAX_TASKS)
            task = 0;
        ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
    }
    return task;
    }

}



bool createThread(_fn fn, const char name[], uint8_t priority, uint32_t stackBytes)
{

    bool ok = false;
    uint8_t i = 0,j=0,len=0;
    bool found = false;
    // REQUIRED: store the thread name
    // add task if room in task list
    // allocate stack space for a thread and assign to sp below-done

    if (taskCount < MAX_TASKS)
    {
        // make sure fn not already in list (prevent reentrancy)
        while (!found && (i < MAX_TASKS))
        {
            found = (tcb[i++].pid ==  fn);
        }
        if (!found)
        {
            // find first available tcb record
            i = 0;
            while (tcb[i].state != STATE_INVALID) {i++;}
            tcb[i].state = STATE_UNRUN;
            tcb[i].pid = fn;
            tcb[i].sp = &stack[i][511];
            tcb[i].spInit=tcb[i].sp;
            tcb[i].priority = priority;
            tcb[i].currentPriority = priority;
            for (j = 0; name[j] != '\0'; j++)
                            {
                                len++;
                            }
                 for( j=0;j<len;j++)
                 {
                 tcb[i].name[j]=name[j];

                 }
            // increment task count
            taskCount++;
            ok = true;
        }
    }
    // REQUIRED: allow tasks switches again
    return ok;
}

// REQUIRED: modify this function to restart a thread
void restartThread(uint8_t num)
{
    __asm(" SVC #18");

}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
void destroyThread(_fn fn)
{
    __asm(" SVC #03");


}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
    __asm(" SVC #21");


}

struct semaphore* createSemaphore(uint8_t semi,uint8_t count, const char name[])
{
    uint8_t i;
    struct semaphore *pSemaphore = 0;
    if (semi < MAX_SEMAPHORES)
    {
        pSemaphore = &semaphores[semi];
        pSemaphore->count = count;

        for(i=0;name[i]!='\0';i++)
        {
            pSemaphore->name[i]=name[i];

        }

    }


    return pSemaphore;
}

// REQUIRED: modify this function to start the operating system, using all created tasks
void startRtos()
{

    //Initialize systickISR
    NVIC_ST_RELOAD_R = 0x00009C3F;
    NVIC_ST_CURRENT_R =NVIC_ST_CURRENT_M;
    NVIC_ST_CTRL_R = NVIC_ST_CTRL_COUNT | NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_INTEN | NVIC_ST_CTRL_ENABLE;

    taskCurrent=rtosScheduler();
    setPSP(tcb[taskCurrent].sp);

    // Enable clocks
         SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;

    // Configure Timer 1 as the time base
               TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
               TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
               TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD | TIMER_TAMR_TACDIR;          // configure for periodic mode (count down)
               TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer

               _fn fn;
               fn=(_fn)tcb[taskCurrent].pid ;

               NVIC_MPU_NUMBER_R =3;//flash
                  NVIC_MPU_BASE_R =0;
                  NVIC_MPU_ATTR_R =0;
                  NVIC_MPU_ATTR_R |=(0b011<<24)| (0b010<<16) |(17<<1)|1;

                  NVIC_MPU_NUMBER_R =0;//entire memory
                     NVIC_MPU_BASE_R =0;
                     NVIC_MPU_ATTR_R =0;
                     NVIC_MPU_ATTR_R |=(1<<28)|(0b011<<24)| (0b111<<16) |(0b11111<<1)|1;


              if(taskCurrent<8)
              {
                     NVIC_MPU_NUMBER_R =7;//sram mpu1
                        NVIC_MPU_BASE_R =0x20000000;
                        NVIC_MPU_ATTR_R =0;
                        NVIC_MPU_ATTR_R |=(1<<28)|(0b001<<24)| (0b110<<16)| ((1<<taskCurrent)<<8)|(13<<1)|1;
                        NVIC_MPU_NUMBER_R =6;//sram mpu2
                                           NVIC_MPU_BASE_R =0x20004000;
                                           NVIC_MPU_ATTR_R =0;
                                           NVIC_MPU_ATTR_R |=(1<<28)|(0b001<<24)| (0b110<<16)| (0b00000000<<8)|(13<<1)|1;
              }

              else if(taskCurrent==8)
              {
                  NVIC_MPU_NUMBER_R =7;//sram mpu1
                           NVIC_MPU_BASE_R =0x20000000;
                           NVIC_MPU_ATTR_R =0;
                           NVIC_MPU_ATTR_R |=(1<<28)|(0b001<<24)| (0b110<<16)| (0b00000000<<8)|(13<<1)|1;
                        NVIC_MPU_NUMBER_R =6;//sram mpu2
                                  NVIC_MPU_BASE_R =0x20004000;
                                  NVIC_MPU_ATTR_R =0;
                                  NVIC_MPU_ATTR_R |=(1<<28)|(0b001<<24)| (0b110<<16)| (0b00000001<<8)|(13<<1)|1;

              }
                     NVIC_SYS_HND_CTRL_R |= (0b111<<16);
                     NVIC_MPU_CTRL_R |=1;


                  setTMPL();



     (*fn)();

    return;
}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
// push registers, call scheduler, pop registers, return to new function
void yield()
{
   __asm(" SVC #02");
    return;
}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
// push registers, set state to delayed, store timeout, call scheduler, pop registers,
// return to new function (separate unrun or ready processing)
void sleep(uint32_t tick)
{
    __asm(" SVC #4");
       return;
}

// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler using pendsv
void wait(int pSemaphore)
{
    __asm(" SVC #6");
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(int pSemaphore)
{
    __asm(" SVC #1");
}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{
uint8_t i;
uint32_t sum[MAX_TASKS];
uint32_t sum1[MAX_TASKS];
uint32_t percent[MAX_TASKS];
uint32_t point[MAX_TASKS];
    for(i=0;i<=MAX_TASKS;i++)

    {
    if(tcb[i].state==STATE_DELAYED)
     {
        tcb[i].ticks--;
    if((tcb[i].ticks)==0)
      {
        tcb[i].state=STATE_READY;
      }
     }
    }

    //ping-pong
    j1.j++;
    if(j1.j==1000)
    {
        j1.j=0;
for(i=0;i<9;i++)
{
    tcb[i].flag=1;
    tcb[i].sum[1]=tcb[i].sum[0];
}
        for(i=0;i<9;i++)
        {


            time[i].sum= tcb[0].sum[1] + tcb[1].sum[1] + tcb[2].sum[1] + tcb[3].sum[1] + tcb[4].sum[1] + tcb[5].sum[1]+ tcb[6].sum[1] + tcb[7].sum[1] + tcb[8].sum[1];

        sum[i]=time[i].sum;
        sum1[i]= tcb[i].sum[1];


        percent[i]= ((sum1[i]*100)/(sum[i]));
        point[i]=((sum1[i]*10000)/(sum[i]))%100;
        time[i].task_percent=percent[i];
        time[i].task_point=point[i];
        }
    }

//enable preemption
if(shell1.preempt)
{
    NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;
}

}



// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{
    if((NVIC_FAULT_STAT_R & 1) || (NVIC_FAULT_STAT_R & 2))
        {
            putsUart0("\n \r called from MPU \n");
            NVIC_FAULT_STAT_R |= 3;
        }

    pushR4_11_PSP();

    tcb[taskCurrent].sp=getPSP();

    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;       //turn off timer1
    time[taskCurrent].t1=TIMER1_TAV_R;
    TIMER1_TAV_R=0;

    time[taskCurrent].deltat=time[taskCurrent].t1-time[taskCurrent].t2;
    tcb[taskCurrent].flag=0;
    tcb[taskCurrent].sum[0]=time[taskCurrent].deltat ;

    taskCurrent=rtosScheduler();
    if(taskCurrent<8)
        {
               NVIC_MPU_NUMBER_R =7;//sram mpu1
                  NVIC_MPU_BASE_R =0x20000000;
                  NVIC_MPU_ATTR_R =0;
                  NVIC_MPU_ATTR_R |=(1<<28)|(0b001<<24)| (0b110<<16)| ((1<<taskCurrent)<<8)|(13<<1)|1;
                  NVIC_MPU_NUMBER_R =6;//sram mpu2
                                     NVIC_MPU_BASE_R =0x20004000;
                                     NVIC_MPU_ATTR_R =0;
                                     NVIC_MPU_ATTR_R |=(1<<28)|(0b001<<24)| (0b110<<16)| (0b00000000<<8)|(13<<1)|1;
        }
        else if(taskCurrent==8)
        {
            NVIC_MPU_NUMBER_R =7;//sram mpu1
                     NVIC_MPU_BASE_R =0x20000000;
                     NVIC_MPU_ATTR_R =0;
                     NVIC_MPU_ATTR_R |=(1<<28)|(0b001<<24)| (0b110<<16)| (0b00000000<<8)|(13<<1)|1;
                  NVIC_MPU_NUMBER_R =6;//sram mpu2
                            NVIC_MPU_BASE_R =0x20004000;
                            NVIC_MPU_ATTR_R =0;
                            NVIC_MPU_ATTR_R |=(1<<28)|(0b001<<24)| (0b110<<16)| (0b00000001<<8)|(13<<1)|1;
        }

    TIMER1_CTL_R |= TIMER_CTL_TAEN;
    time[taskCurrent].t2=TIMER1_TAV_R;

    setPSP1(tcb[taskCurrent].sp);

   if(tcb[taskCurrent].state==STATE_READY)
     {
      popR4_11_PSP();
    }
    else //UNRUN
    {
      setPSP1(tcb[taskCurrent].sp);
      pushR0_xPSR(tcb[taskCurrent].pid);
      tcb[taskCurrent].state=STATE_READY;
      exc_ret();


    }

}

bool cmp(char str1[], char str2[])
{

    uint8_t i=0;
    while(str1[i]!='\0' || str2[i]!='\0')
    {
        if(str1[i]==str2[i] || str1[i]-str2[i]==32)
        {
            i++;
            continue;
        }
        else

            return false;

    }
    return true;


}


uint8_t getSVCNo()
{
   uint32_t **pr1= getPSP();
   pr1=pr1+6;
   uint32_t *pr2;
   pr2=*pr1;
   uint32_t x=(uint32_t)pr2;
   x=x-2;
   pr2=(uint32_t *)x;
   uint8_t k=*pr2;

return k;
}
// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void svCallIsr()
{

 uint8_t n=getSVCNo();
uint32_t x=getR0();
uint32_t r;
char *pidof=(char*)getR0();
struct semaphore *s;
s=&semaphores[x];

uint8_t i,j,k,l;
uint32_t *r0=getPSP();
struct semaphore *y;
struct semaphore *z;
switch(n)
 {


case 02://yield
       NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;
       break;
case 04://sleep
    tcb[taskCurrent].state=STATE_DELAYED;

    tcb[taskCurrent].ticks=*r0;

    NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;
    break;
case 06://wait



    if (s->count >0)
    {
        s->count--;
        s->currTask=taskCurrent;

    }
    else
    {

        s->processQueue[s->queueSize]=taskCurrent;
        tcb[s->processQueue[s->queueSize]].state=STATE_BLOCKED;
        tcb[s->processQueue[s->queueSize]].semaphore=s;
        if(shell1.pi)
               {


            for(k=0;k<9;k++)
            {
                if(tcb[taskCurrent].semaphore==tcb[k].semaphore)
                {
                    if(tcb[taskCurrent].currentPriority<tcb[k].currentPriority)
                    {
                        tcb[k].currentPriority=tcb[taskCurrent].currentPriority;
                    }
                }
            }

               }

        s->queueSize++;
        s->currTask=taskCurrent;

        NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;

    }
    break;

case 01://post
    s->count++;
    i=0,j=0;

        while(s->processQueue[i]!=0)
              {
            tcb[s->processQueue[i]].state=STATE_READY;
            if(shell1.pi)
            {
                for(k=0;k<9;k++)
                           {
                               if(tcb[taskCurrent].semaphore==tcb[k].semaphore )
                               {
                                   if(tcb[taskCurrent].currentPriority==tcb[k].currentPriority)
                                   {
                                       tcb[taskCurrent].currentPriority=tcb[taskCurrent].priority;
                                   }
                               }
                           }

            }

            for(j=0;j<MAX_QUEUE_SIZE;j++)
            {

            if(j==MAX_QUEUE_SIZE-1)
             {
                s->processQueue[j]=0;
             }
            else
             {
            s->processQueue[j]=s->processQueue[j+1];
             }
            }
            s->queueSize--;
            s->count--;

              }


    break;

case 03: //kill
   i=0;
    j=0;
    l=0;

    for(i=0;i<9;i++)
    {
    if(x==(uint32_t)tcb[i].pid)
     {
        if(x==(uint32_t)tcb[0].pid)
        {
            tcb[i].state=tcb[i].state;
        }
        else
        {
        tcb[i].state=STATE_INVALID;

        }
        y=tcb[i].semaphore;
        if(tcb[i].semaphore!=0)
        {

            for(j=0;j<MAX_QUEUE_SIZE;j++)
               {
                if(x==(uint32_t)tcb[y->processQueue[j]].pid)
                {

                  if(j==MAX_QUEUE_SIZE-1)
                    {
                      y->processQueue[j]=0;
                    }
                   else
                   {
                    y->processQueue[j]=y->processQueue[j+1];
                    if(y->queueSize==0)
                               {
                                   y->queueSize=0;
                               }
                               else
                               {
                               y->queueSize--;
                               }

                    }
                }

                //condition:for imporatant and lengthyfn kill and restart start here
                for(k=0;k<9;k++)
                {

                    if(tcb[y->processQueue[l]].pid==tcb[k].pid && y->processQueue[l]!=0)
                        {
                        if(tcb[y->processQueue[l]].state !=STATE_INVALID)
                        {
                    tcb[y->processQueue[l]].state =STATE_READY;
                        }
                        else
                        {
                            if(y->count ==0)
                                                   {
                                                   y->count++;
                                                   }
                        }
                    if(shell1.pi)
                               {
                                   for(k=0;k<9;k++)
                                              {
                                                  if(tcb[i].semaphore==tcb[y->processQueue[l]].semaphore)
                                                  {
                                                      if(tcb[i].currentPriority==tcb[y->processQueue[l]].currentPriority)
                                                      {
                                                          tcb[y->processQueue[l]].currentPriority=tcb[y->processQueue[l]].priority;
                                                      }
                                                  }
                                              }

                               }
                    if(y->processQueue[l]!=STATE_INVALID)
                    {
                    for(j=0;j<MAX_QUEUE_SIZE;j++)
                               {

                               if(j==MAX_QUEUE_SIZE-1)
                                {
                                   y->processQueue[j]=0;
                                }
                               else
                                {
                               y->processQueue[j]=y->processQueue[j+1];
                                }
                               }
                    if(y->queueSize==0)
                                                   {
                                                       y->queueSize=0;
                                                   }
                                                   else
                                                   {
                                                   y->queueSize--;
                                                   }

                     }
                   }
                else if(tcb[k].semaphore==tcb[i].semaphore && k!=i)
                {
                    if(tcb[k].state !=STATE_INVALID)
                    {
                    tcb[k].state=STATE_READY;
                    }
                    else
                    {
                        if(y->count ==0)
                        {
                        y->count++;
                        }

                    }


                }
                }
                //condition for important and lengthyfn for kill and restart end here

                //condition for readkeys
                if(i==4)
                {
                  z=tcb[i+1].semaphore;
                  if(tcb[z->processQueue[l]].state !=STATE_INVALID &&  z->processQueue[l]!=0)
                            {
                               tcb[z->processQueue[l]].state =STATE_READY;
                             }
                     else
                     {
                       if(z->count ==0)
                         {
                               z->count++;
                         }
                     }
                        if(shell1.pi)
                            {
                                  for(k=0;k<9;k++)
                                    {
                                         if(tcb[i].semaphore==tcb[y->processQueue[l]].semaphore)
                                           {
                                              if(tcb[i].currentPriority==tcb[y->processQueue[l]].currentPriority)
                                                {
                                                    tcb[y->processQueue[l]].currentPriority=tcb[y->processQueue[l]].priority;
                                                }
                                             }
                                     }

                               }
                        if(z->processQueue[l]!=STATE_INVALID)
                        {
                        for(j=0;j<MAX_QUEUE_SIZE;j++)
                          {

                              if(j==MAX_QUEUE_SIZE-1)
                                 {
                                    z->processQueue[j]=0;
                                 }
                              else
                                {
                                  z->processQueue[j]=y->processQueue[j+1];
                                 }
                             }
                              if(z->queueSize==0)
                               {
                                 z->queueSize=0;
                               }
                               else
                               {
                                  z->queueSize--;
                               }
                        }
                }
                //condition for readkeys for kill and restart end here




                }

        }



     }

    }
    break;




case 10://ps
     {
         putsUart0("\r \n");
                        putsUart0("PID    Process_Name  Priority    Status \t \t    CPU TIME(%)");
                        putsUart0("\r \n");

                        for (i=0;i<9;i++)
                        {
                             z=tcb[i].semaphore;
                           itoa((uint32_t)tcb[i].pid,shell1.strout,1);
                           putsUart0(shell1.strout);
                           putsUart0("\t");

                           putsUart0(tcb[i].name);
                           putsUart0("     \t");
                           itoa((uint8_t)tcb[i].currentPriority,shell1.strout,1);
                           putsUart0(shell1.strout);

                           putsUart0("     \t");

                           if(tcb[i].state==0)
                           {
                               putsUart0("INVALID");
                               putsUart0("\t\t");
                           }
                           else if(tcb[i].state==1)
                           {
                               putsUart0("UNRUN");
                               putsUart0("    \t");
                           }
                           else if(tcb[i].state==2)
                           {
                               putsUart0("READY");
                               putsUart0("     \t");
                           }
                           else if(tcb[i].state==3)
                           {

                               putsUart0("SLEEP for ");
                               itoa(tcb[i].ticks,shell1.strout,1);
                               putsUart0(shell1.strout);
                               putsUart0(" ticks");

                           }
                           else
                           {
                               putsUart0("BLOCKED by ");
                               putsUart0(z->name);
                           }

                           putsUart0(" \t  \t");
                           itoa(time[i].task_percent,shell1.strout,1);
                           putsUart0(shell1.strout);
                           putsUart0(".");
                           itoa(time[i].task_point,shell1.strout,1);
                           putsUart0(shell1.strout);
                           putsUart0("\r \n");

                        }
                        break;
                           }

case 12://ipcs
{

    putsUart0("\r \n");
                                putsUart0(" Semaphore_Name   Wait_Count     Count ");
                                putsUart0("\r \n");

                                    for (i=0;i<4;i++)
                                 {

                                     putsUart0(semaphores[i].name);

                                     putsUart0("          \t");


                                     itoa(semaphores[i].queueSize,shell1.strout,1);

                                     putsUart0(shell1.strout);
                                     putsUart0("           ");


                                     itoa(semaphores[i].count,shell1.strout,1);

                                     putsUart0(shell1.strout);
                                     putsUart0("         ");

                                     putsUart0("\r \n");

                                 }
                                    break;
}

case 14://pidof
{

    putsUart0("\r \n");
                    if( cmp("idle",pidof)) //Command: pidof idle
                      {
                        itoa((uint32_t)tcb[0].pid,shell1.strout,1);
                        putsUart0(shell1.strout);
                        putsUart0("\r \n");
                      }
                    else if( cmp("lenghtyfn",pidof)) //Command: pidof lengthyfn
                       {
                         itoa((uint32_t)tcb[1].pid,shell1.strout,1);
                         putsUart0(shell1.strout);
                         putsUart0("\r \n");
                       }
                    else if( cmp("flash4hz",pidof)) //Command: pidof flash4hz
                        {
                          itoa((uint32_t)tcb[2].pid,shell1.strout,1);
                          putsUart0(shell1.strout);
                          putsUart0("\r \n");
                        }
                    else if( cmp("oneshot",pidof)) //Command: pidof oneshot
                        {
                          itoa((uint32_t)tcb[3].pid,shell1.strout,1);
                          putsUart0(shell1.strout);
                          putsUart0("\r \n");
                        }
                    else if( cmp("readkeys",pidof)) //Command: pidof readkeys
                        {
                          itoa((uint32_t)tcb[4].pid,shell1.strout,1);
                          putsUart0(shell1.strout);
                          putsUart0("\r \n");
                        }
                    else if( cmp("debounce",pidof)) //Command: pidof debounce
                        {
                          itoa((uint32_t)tcb[5].pid,shell1.strout,1);
                          putsUart0(shell1.strout);
                          putsUart0("\r \n");
                        }
                    else if( cmp("important",pidof)) //Command: pidof important
                        {
                          itoa((uint32_t)tcb[6].pid,shell1.strout,1);
                          putsUart0(shell1.strout);
                          putsUart0("\r \n");
                        }
                    else if( cmp("uncoop",pidof)) //Command: pidof uncoop
                        {
                          itoa((uint32_t)tcb[7].pid,shell1.strout,1);
                          putsUart0(shell1.strout);
                          putsUart0("\r \n");
                        }
                    else if( cmp("shell",pidof)) //Command: pidof shell
                        {
                          itoa((uint32_t)tcb[8].pid,shell1.strout,1);
                          putsUart0(shell1.strout);
                          putsUart0("\r \n");
                        }
                    else
                        putsUart0("Invalid command \r \n");

                    break;
}

case 15://pi
{
    shell1.pi=x;
    break;
}

case 17://preempt
{
    shell1.preempt=x;
    break;
}

case 18://restart
{
    _fn pid=(_fn)tcb[x].pid;
       uint8_t i;
       for(i=0;i<9;i++)
       {
           if((_fn)tcb[i].pid==pid)
           {
              tcb[i].state=STATE_UNRUN;
              tcb[i].sp=tcb[i].spInit;
              break;
           }

       }
       break;
}

case 19://sched
{
    shell1.mode=x;
    if(shell1.mode==1)
    {
        time[0].deltat=0;
                                               time[0].sum=0;
                                               time[0].t1=0;
                                               time[0].t2=0;
                                               time[0].task_percent=0;
                                               time[0].task_point=0;
                                               tcb[0].sum[0]=0;
                                               tcb[0].sum[1]=0;
    }
        break;
}

case 20://reboot
{
    NVIC_APINT_R= NVIC_APINT_VECTKEY|NVIC_APINT_SYSRESETREQ |NVIC_APINT_VECT_RESET;
    break;
}
case 21://setThreadPriority
{
    r=getR1();
    _fn pid=(_fn)x;

        for(i=0;i<9;i++)
        {
            if((_fn)tcb[i].pid==pid)
            {
                tcb[i].currentPriority=r;
                break;
            }

        }
        break;
}

 }
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
// REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
//           6 pushbuttons, and uart
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, sysdivider of 5, creating system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOD ;

    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R = BLUE_LED_MASK;  // bits 1 and 3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = BLUE_LED_MASK | RED_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R =  BLUE_LED_MASK ;  // enable LEDs and pushbuttons


    // Configure LED and pushbutton pins
        GPIO_PORTA_DIR_R = RED_LED_MASK | ORANGE_LED_MASK | YELLOW_LED_MASK;  // bits 1 and 3 are outputs, other pins are inputs
        GPIO_PORTA_DR2R_R = RED_LED_MASK | ORANGE_LED_MASK | YELLOW_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
        GPIO_PORTA_DEN_R =  RED_LED_MASK | ORANGE_LED_MASK | YELLOW_LED_MASK;  // enable LEDs and pushbuttons


        GPIO_PORTE_DIR_R = GREEN_LED_MASK;  // bits 1 and 3 are outputs, other pins are inputs
         GPIO_PORTE_DR2R_R = GREEN_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
         GPIO_PORTE_DEN_R =  GREEN_LED_MASK;  // enable LEDs and pushbuttons

         GPIO_PORTC_DEN_R = PUSH_BUTTON0_MASK | PUSH_BUTTON1_MASK | PUSH_BUTTON2_MASK | PUSH_BUTTON3_MASK;  // enable LEDs and pushbuttons
         GPIO_PORTC_PUR_R = PUSH_BUTTON0_MASK | PUSH_BUTTON1_MASK | PUSH_BUTTON2_MASK | PUSH_BUTTON3_MASK; // enable internal pull-up for push button

        GPIO_PORTD_LOCK_R = 0x4C4F434B;
        GPIO_PORTD_CR_R=0x80;
        GPIO_PORTD_DEN_R = PUSH_BUTTON4_MASK | PUSH_BUTTON5_MASK;  // enable LEDs and pushbuttons
        GPIO_PORTD_PUR_R = PUSH_BUTTON4_MASK | PUSH_BUTTON5_MASK ; // enable internal pull-up for push button
        GPIO_PORTD_CR_R=0x00;

        // Configure UART0 pins
             GPIO_PORTA_DIR_R |= 2;                           // enable output on UART0 TX pin: default, added for clarity
             GPIO_PORTA_DEN_R |= 3;                           // enable digital on UART0 pins: default, added for clarity
             GPIO_PORTA_AFSEL_R |= 3;                         // use peripheral to drive PA0, PA1: default, added for clarity
             GPIO_PORTA_PCTL_R &= 0xFFFFFF00;                 // set fields for PA0 and PA1 to zero
             GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
                                                              // select UART0 to drive pins PA0 and PA1: default, added for clarity

             // Configure UART0 to 115200 baud, 8N1 format
             SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other UARTs in same status
             delay4Cycles();                                  // wait 4 clock cycles
             UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
             UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
             UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
             UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
             UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
             UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                              // enable TX, RX, and module
}

// REQUIRED: add code to return a value from 0-63 indicating which of 6 PBs are pressed-done
uint8_t readPbs()
{
           if(PUSH_BUTTON0==0)
            {
              return 1;
            }
            if(PUSH_BUTTON1==0)
            {
              return 2;
            }
            if(PUSH_BUTTON2==0)
            {
               return 4;
            }
            if(PUSH_BUTTON3==0)
            {
              return 8;
            }
            if(PUSH_BUTTON4==0)
            {
                return 16;
            }
            if(PUSH_BUTTON5==0)
            {
                return 32;
            }

    return 0;
}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{

    while(true)
       {
           ORANGE_LED = 1;
           waitMicrosecond(1000);
           ORANGE_LED = 0;
           yield();
       }
}

void flash4Hz()
{
    while(true)
       {
           GREEN_LED ^= 1;
           sleep(125);
       }
}

void oneshot()
{
    while(true)
        {
            wait(flashReq);
            YELLOW_LED = 1;
            sleep(1000);
            YELLOW_LED = 0;
        }
}

void partOfLengthyFn()
{
    // represent some lengthy operation
        waitMicrosecond(990);
        // give another process a chance to run
        yield();
}

void lengthyFn()
{
    uint16_t i;
       while(true)
       {
           wait(resource);
           for (i = 0; i < 5000; i++)
           {
               partOfLengthyFn();
           }
           RED_LED ^= 1;
           post(resource);
       }
}

void readKeys()
{
    uint8_t buttons;
       while(true)
       {
           wait(keyReleased);
           buttons = 0;
           while (buttons == 0)
           {
               buttons = readPbs();
               yield();
           }
           post(keyPressed);
           if ((buttons & 1) != 0)
           {
               YELLOW_LED ^= 1;
               RED_LED = 1;
           }
           if ((buttons & 2) != 0)
           {
               post(flashReq);
               RED_LED = 0;
           }
           if ((buttons & 4) != 0)
           {
               restartThread(2);
           }
           if ((buttons & 8) != 0)
           {
//               shell1.pid=(uint32_t)tcb[2].pid;
               destroyThread(flash4Hz);
           }
           if ((buttons & 16) != 0)
           {
               setThreadPriority(lengthyFn, 4);
           }
           yield();
       }
}

void debounce()
{
    uint8_t count;
        while(true)
        {
            wait(keyPressed);
            count = 10;
            while (count != 0)
            {
                sleep(10);
                if (readPbs() == 0)
                    count--;
                else
                    count = 10;
            }
            post(keyReleased);
        }
}

//void uncooperative()
//{
//    while(true)
//        {
//            while (readPbs() == 32)
//            {
//            }
//            yield();
//        }
//}

void errant()
{
uint32_t* p = (uint32_t*)0x20007FFC;
while(true)
{
while (readPbs() == 32)
{
*p = 0;
}
yield();
}
}

void important()
{
    while(true)
    {
        wait(resource);
        BLUE_LED = 1;
        sleep(1000);
        BLUE_LED = 0;
        post(resource);
    }
}

void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF)               // wait if uart0 tx fifo full
    {
//        yield();
    }
       UART0_DR_R = c;                                  // write character to fifo
}



void putsUart0(char* str)
{
    uint8_t i = 0;
    while (str[i] != '\0')
        putcUart0(str[i++]);
}
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE)               // wait if uart0 rx fifo empty
    {
        yield();
    }
      return UART0_DR_R & 0xFF;                        // get character from fifo
}

char* getString(char str[],uint8_t MAX)
{

   int cnt;
   char c;
   for (cnt=0;cnt<MAX;cnt++)

   {
       if(cnt<MAX)
       {
       c=getcUart0();
       if(c==8||c==127)
       {
           if(cnt>0)
               cnt--;
           cnt--;
           continue;
       }
       if(c==10 ||c==13)
       {
           str[cnt]=0;
           break;
       }
       if(c>=' ')
       {
           str[cnt]=c;
       }
       }
       else
       {
           str[cnt]=0;
       break;
       }


 }
   return str;
}

char* parseString(char str[],uint8_t pos[],uint8_t maxField,uint8_t *argCnt,uint8_t *min_args)
{
     *argCnt=0;
    int i=0;
    int j=0;
    int len=0;
    for (i = 0; str[i] != '\0'; i++)
           {
               len++;
           }


    for ( i=0;i<=len;i++)
    {
        if((str[i]>=48 && str[i]<=57)||(str[i]>=65 && str[i]<=90)||( str[i]>=97 && str[i]<=122)||( str[i]==46)||(str[i]==45) || (str[i]==38))
        {
            if(!((str[i-1]>=48 && str[i-1]<=57)||(str[i-1]>=65 && str[i-1]<=90)||( str[i-1]>=97 && str[i-1]<=122)||( str[i-1]==46)||(str[i-1]==45)))
            {

                pos[j]=i;
                j++;
                (*argCnt)++;
                if (*argCnt==maxField)
                {
                    break;
                }
            }
        }
         else if(!((str[i]>=48 && str[i]<=57)||(str[i]>=65 && str[i]<=90)||( str[i]>=97 && str[i]<=122)||( str[i]==46)||(str[i]==45)))
         {

                   str[i]=0;

         }

}

    *min_args=(*argCnt)-1;
return str;
}

char* getArgString(char str[],uint8_t posi[],uint8_t argNum)
{

        return &str[posi[argNum]];

}



bool isCommand(char strCmd[],uint8_t minArgs,uint8_t argCount,char str[],uint8_t position[])
{

           if(cmp(strCmd,getArgString(str,position,0)))
           {
               if(minArgs<argCount)
               {
                   return true;
               }
           }
   return false;
}

//reffered from geeksforgeeks
void reverse(char* s, uint8_t l)
{
    uint8_t i=0,j=l-1;
    char temp;
    for(i=0;i<j;i++)
    {
       temp=s[i];
       s[i]=s[j];
       s[j]=temp;
       j--;
    }
}
//reffered from geeksforgeeks
uint8_t itoa(uint16_t n, char s1[], uint8_t d)
{
    uint8_t i=0;
    while(n)
    {
        s1[i++]=n%10+'0';
        n=n/10;
    }
    while(i<d)
    {
        s1[i++]='0';
    }
    reverse(s1,i);
    s1[i]='\0';
    return i;
}


//referred from geeksforgeeks
int atoi(char* str)
{
    uint32_t res = 0; // Initialize result
    uint16_t i;
    // Iterate through all characters of input string and update result
    for (i = 0; str[i] != '\0'; ++i)
        res = res * 10 + str[i] - '0';

    // return result.
    return res;
}

// REQUIRED: add processing for the shell commands through the UART here
//           your solution should not use C library calls for strings, as the stack will be too large

void pidof(char* str)
{
    __asm(" SVC #14");
}
void pi(uint8_t i)
{
    __asm(" SVC #15");
}
void preempt(uint8_t i)
{
    __asm(" SVC #17");
}
void sched(uint8_t i)
{
    __asm(" SVC #19");
}
void shell()
{
//uint8_t i;
//struct semaphore *z;
char str[MAX_CHARS+1];
    uint8_t position[MAX_FIELDS];
    uint8_t argCount;

    uint8_t min_args;
//    char strout[MAX_CHARS];

    while (true)
    {


            putsUart0("Enter any string\r\n");
            putsUart0("\r \n");

            getString(str,MAX_CHARS);
            parseString(str,position,MAX_FIELDS, &argCount,&min_args);

            //Display PID,Process_Name,Priority,Status and CPU time in percentage

            if(isCommand("ps",0,argCount,str,position))     //Command: ps
                {
                __asm(" SVC #10");
                }
            //Display Semaphore_Name,wait_cpunt and count
                       if(isCommand("ipcs",0,argCount,str,position))      //Command: ipcs
                          {
                           __asm(" SVC #12");
                          }


            //Display PID of Process_Name
            if(isCommand("pidof",1,argCount,str,position))         //Command: pidof <Process_Name>
            {
                pidof(getArgString(str,position,1));

            }

            //kill the Process of the given PID
            if(isCommand("kill",1,argCount,str,position))          //Command: kill <PID>
               {


                  putsUart0("\r \n");
                  uint32_t id= atoi(getArgString(str,position,1));
                  destroyThread((_fn)id);


                }

            //turn on or off priority inheritance
            if(isCommand("pi",1,argCount,str,position))          //Command: pi <on/off>
            {
                if( cmp("on",getArgString(str,position,1))) //Command: pi on
                {
                    putsUart0("\r \n");
                    pi(1);


                 }
                else if( cmp("off",getArgString(str,position,1))) //Command: pi off
                {
                    putsUart0("\r \n");
                    pi(0);


                }
                else
                {
                    putsUart0("Invalid Command");
                }
            }

            //turn on or off preemption
            if(isCommand("preempt",1,argCount,str,position))       //Command: preempt <on/off>
              {
                if( cmp("on",getArgString(str,position,1))) //Command: preempt on
                  {
                    putsUart0("\r \n");
                    preempt(1);


                  }
               else if( cmp("off",getArgString(str,position,1))) //Command: preempt off
                  {
                   putsUart0("\r \n");
                   preempt(0);

                  }
               else
               {
                   putsUart0("Invalid Command");
               }
               }

//Restart the thread that is killed
//Command: <Process_Name> &
            if(isCommand("lengthyfn",1,argCount,str,position)) //Command: lengthyfn &
            {
                if( cmp("&",getArgString(str,position,1)))
                {

//                if(tcb[1].state==STATE_INVALID)
//                               {
                putsUart0("\r \n");
                restartThread(1);
//                               }
                }
            }
            if(isCommand("flash4hz",1,argCount,str,position)) //Command: flash4hz &
                        {
                if( cmp("&",getArgString(str,position,1)))
                {
//                if(tcb[2].state==STATE_INVALID)
//                {
                            putsUart0("\r \n");
                            restartThread(2);
//                }
                }
                        }
            if(isCommand("oneshot",1,argCount,str,position)) //Command: oneshot &
                        {
                if( cmp("&",getArgString(str,position,1)))
                {
//                if(tcb[3].state==STATE_INVALID)
//                               {
                            putsUart0("\r \n");
                            restartThread(3);
//                               }
                 }
                        }
            if(isCommand("readkeys",1,argCount,str,position)) //Command: readkeys &
                        {
                if( cmp("&",getArgString(str,position,1)))
                {
//                if(tcb[4].state==STATE_INVALID)
//                               {
                            putsUart0("\r \n");
                            restartThread(4);
//                               }
                }
                        }
            if(isCommand("debounce",1,argCount,str,position)) //Command: debounce &
                        {
                if( cmp("&",getArgString(str,position,1)))
                {
//                if(tcb[5].state==STATE_INVALID)
//                               {
                            putsUart0("\r \n");
                            restartThread(5);
//                               }
                }
                        }
            if(isCommand("important",1,argCount,str,position)) //Command: important &
                        {
                if( cmp("&",getArgString(str,position,1)))
                {
//                if(tcb[6].state==STATE_INVALID)
//                               {
                            putsUart0("\r \n");
                            restartThread(6);
//                               }
                }
                        }
            if(isCommand("errant",1,argCount,str,position)) //Command: uncoop &
                        {
                if( cmp("&",getArgString(str,position,1)))
                {
//                if(tcb[7].state==STATE_INVALID)
//                               {
                            putsUart0("\r \n");
                            restartThread(7);
//                               }
                }
                        }
            if(isCommand("shell",1,argCount,str,position))//Command: shell &
                        {
                if( cmp("&",getArgString(str,position,1)))
                {
//                if(tcb[8].state==STATE_INVALID)
//                               {
                            putsUart0("\r \n");
                            restartThread(8);
//                               }
                }
                        }

            //to change the type of schedular between priorty schedular(priority)and round robin(rr) schedular
            if(isCommand("sched",1,argCount,str,position))       //Command: sched <rr/priority>
            {
                if( cmp("rr",getArgString(str,position,1))) //Command: sched rr
                                  {
                    putsUart0("\r \n");
                                      sched(0);


                                  }
                               else  if( cmp("priority",getArgString(str,position,1))) //Command: sched priority
                                  {
                                   putsUart0("\r \n");
                                   sched(1);
                                  }
            }

            //to restart the processor
            if(isCommand("reboot",0,argCount,str,position)) //Command: reboot
                  {
                putsUart0("\r \n");
//                NVIC_APINT_R= NVIC_APINT_VECTKEY|NVIC_APINT_SYSRESETREQ |NVIC_APINT_VECT_RESET;
                __asm(" SVC #20");


                  }

            if(!(isCommand("ps",0,argCount,str,position) || isCommand("ipcs",0,argCount,str,position) || isCommand("pidof",1,argCount,str,position) ||  isCommand("kill",1,argCount,str,position)  || isCommand("pi",1,argCount,str,position)  ||  isCommand("preempt",1,argCount,str,position) || isCommand("flash4hz",1,argCount,str,position)  ||  isCommand("lengthyfn",1,argCount,str,position)  || isCommand("oneshot",1,argCount,str,position) || isCommand("readkeys",1,argCount,str,position) || isCommand("debounce",1,argCount,str,position) || isCommand("important",1,argCount,str,position) || isCommand("errant",1,argCount,str,position) || isCommand("shell",1,argCount,str,position) || isCommand("sched",1,argCount,str,position) || isCommand("reboot",0,argCount,str,position)))
                    {
                putsUart0(" Invalid command \r \n");
                    }
 }

}
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------
void rev(char str[],uint16_t i)
{
    int s=0,e;
    char temp;

    for(e=i-1;s<e;e--)
    {
        temp=*(str+s);
        *(str+s) = *(str+e);
        *(str+e) = temp;
        s++;
    }
}

char* dectohex(uint32_t dec, char hex[])
{
    int rem,i=0;

    for(i=0;i<9;i++)
    {
      if(i==8)
      {
          hex[i]='\0';
          break;
      }
      if(dec==0)
                   {
                       hex[i]='0';
                   }

     if(dec!=0)
    {
        rem=dec%16;

        if(rem<10)
        {
          hex[i]=rem+48;

        }
        else if(rem>=10)
        {
            hex[i]=rem+55;
        }
        dec=dec/16;
    }


    }

    rev(hex,i);
    return hex;


}
char *i22a(uint32_t num,char *str)
{
    uint8_t i = 0;
//    uint8_t rem = 0;

    if(num == 0)
    {
        str[i] = '0';
        str[++i] = '\0';
        return str;
    }

    while(num)
    {
        uint8_t rem = num % 10;

        if (rem >= 10)
            str[i++] = 65 + (rem - 10);
        else
            str[i++] = 48 + rem;
        num = num / 10;
    }
    str[i] = '\0';
    rev(str,i);
    return str;
}
void mpufaultisr()
{
    char strr[100];
    uint32_t addr;

    putsUart0("\n \r MPU fault in process ");
    i22a((uint32_t)tcb[taskCurrent].pid,strr);
    putsUart0(strr);

    putsUart0("\n \r PSP: ");
    addr=get_PSP();
    dectohex(addr,strr);
    putsUart0(strr);

    putsUart0("\n \r MSP: ");
    addr=get_MSP();
    dectohex(addr,strr);
    putsUart0(strr);

    putsUart0("\n \r mfault flags: ");
    dectohex((NVIC_FAULT_STAT_R & 255),strr);
    putsUart0(strr);
    putsUart0("\n \r Offending instruction address: ");
    dectohex(NVIC_MM_ADDR_R,strr);
    putsUart0(strr);

    putsUart0("\n \r xPSR: ");
    addr=getxPSR();
    dectohex(addr,strr);
    putsUart0(strr);

    putsUart0("\n \r PC: ");
    addr=getPC();
    dectohex(addr,strr);
    putsUart0(strr);

    putsUart0("\n \r LR: ");
    addr=getLR();
    dectohex(addr,strr);
    putsUart0(strr);

    putsUart0("\n \r R0: ");
    addr=getR0();
    dectohex(addr,strr);
    putsUart0(strr);

    putsUart0("\n \r R1: ");
    addr=getR1();
    dectohex(addr,strr);
    putsUart0(strr);

    putsUart0("\n \r R2: ");
    addr=getR2();
    dectohex(addr,strr);
    putsUart0(strr);

    putsUart0("\n \r R3: ");
    addr=getR3();
    dectohex(addr,strr);
    putsUart0(strr);

    putsUart0("\n \r R12: ");
    addr=getR12();
    dectohex(addr,strr);
    putsUart0(strr);


    tcb[taskCurrent].state=STATE_INVALID;
    NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;

}

void hardfaultisr()
{
    char strr[20];
    uint32_t addr;
    putsUart0("Hard fault in process ");
    i22a((uint32_t)tcb[taskCurrent].pid,strr);
    putsUart0(strr);

    putsUart0("\n \r PSP: ");
        addr=get_PSP();
        dectohex(addr,strr);
        putsUart0(strr);

        putsUart0("\n \r MSP: ");
        addr=get_MSP();
        dectohex(addr,strr);
        putsUart0(strr);

        while(1);
}
void usagefaultisr()
{
    char strr[20];
//    uint32_t addr;
    putsUart0("Usage fault in process  ");
    i22a((uint32_t)tcb[taskCurrent].pid,strr);
    putsUart0(strr);

        while(1);
}

void busfaultisr()
{
    char strr[20];
//    uint32_t addr;
    putsUart0("Bus fault in process ");
    i22a((uint32_t)tcb[taskCurrent].pid,strr);
    putsUart0(strr);

        while(1);
}

int main(void)
{
    shell1.argCount=0;

       shell1.min_args=0;
       shell1.mode=1;
       shell1.preempt=true;
       shell1.pi=true;

    bool ok;

    // Initialize hardware
    initHw();
    initRtos();

    // Power-up flash
    GREEN_LED = 1;
    waitMicrosecond(250000);
    GREEN_LED = 0;
    waitMicrosecond(250000);

    // Initialize semaphores
//    keyPressed = createSemaphore(1,"keyPressed");
//    keyReleased = createSemaphore(0,"keyReleased");
//    flashReq = createSemaphore(5,"flashReq");
//    resource = createSemaphore(1,"resource");

    createSemaphore(keyPressed,1,"keyPressed");
       createSemaphore(keyReleased,0,"keyReleased");
       createSemaphore(flashReq,5,"flashReq");
       createSemaphore(resource,1,"resource");

    // Add required idle process at lowest priority
       ok =  createThread(idle, "Idle", 15, 1024);


    // Add other processes
     ok &= createThread(lengthyFn, "LengthyFn", 12, 1024);
       ok &= createThread(flash4Hz, "Flash4Hz", 8, 1024);
       ok &= createThread(oneshot, "OneShot", 4, 1024);
       ok &= createThread(readKeys, "ReadKeys", 12, 1024);
       ok &= createThread(debounce, "Debounce", 12, 1024);
       ok &= createThread(important, "Important", 0, 1024);
       ok &= createThread(errant, "Errant", 12, 1024);
       ok &= createThread(shell, "Shell", 12, 1024);

    // Start up RTOS
    if (ok)
        startRtos(); // never returns
    else
        RED_LED = 1;

    return 0;
}


