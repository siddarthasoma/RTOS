
/* Spring 2014 - EE6314 Group Project 1 - RTOS
 *  Developer :: Siddartha Soma     (UTA)
 */
//-----------------------------------------------------------------------------
// Objectives and notes             
//-----------------------------------------------------------------------------

// Target uC:       33FJ128MC802
// Devices used:    LEDs and PBs

// Hardware description:
// Red LED
//   anode connected through 100ohm resistor to RB5 (pin 14), cathode grounded
// Green LED
//   anode connected through 100ohm resistor to RB4 (pin 11), cathode grounded
// Yellow LED
//   anode connected through 100ohm resistor to RB3 (pin 7), cathode grounded
// Orange LED
//   anode connected through 100ohm resistor to RB2 (pin 6), cathode grounded
// Push Buttons
//   push button 0 connected between RB12 (pin 23) and ground
//   push button 1 connected between RB13 (pin 24) and ground
//   push button 2 connected between RB14 (pin 25) and ground
//   push button 3 connected between RB15 (pin 26) and ground
// SP3232E RS-232 Interface
//  T1IN connected to RP10 (pin 21)
//  R1OUT connected to RP11 (pin 22)

//-----------------------------------------------------------------------------
// Device includes and assembler directives             
//-----------------------------------------------------------------------------
#include <string.h>
#include <p33FJ128MC802.h>
#define FCY 40000000UL                     // instruction cycle rate
#include <libpic30.h>                        // __delay32
#include <stdio.h>                           // __delay_ms (max value is 268)
// __delay_us
#define PIN_YELLOW LATBbits.LATB2            // define i/o
#define PIN_ORANGE LATBbits.LATB3
#define PIN_GREEN LATBbits.LATB4
#define PIN_RED LATBbits.LATB5
#define PIN_PB0 PORTBbits.RB12
#define PIN_PB1 PORTBbits.RB13
#define PIN_PB2 PORTBbits.RB14
#define PIN_PB3 PORTBbits.RB15

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables                
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();

#define TRUE  1
#define FALSE 0
#define MAX_QUEUE_SIZE 10
# define MAX_232_Que 256

//Serial Initialization variables
unsigned int BAUD_38400=64 ;    // brg for low-speed, 40 MHz clock
// round((40000000/16/38400)-1)
unsigned char rx_232_data[MAX_232_Que];
unsigned int write_232_index=0;

// semaphore
int semaphore_count=0;			// Number of semaphores initiated
struct semaphore
{
	unsigned int count; //count of available semaphores(decrement on wait if >0)
	unsigned int queue_size;     //number of processes waiting on the semaphore
	unsigned int process_queue[MAX_QUEUE_SIZE]; // store task index here
} *s, key_pressed, key_released, flash_req, rx_232_dataSemaphore;

// task 
#define STATE_INVALID    0 // no task
#define STATE_READY      1 // ready to run
#define STATE_BLOCKED    2 // has run, but now blocked by semaphore
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define MAX_TASKS 10       // maximum number of valid tasks
int task_current = 0;      // index of last dispatched task
int task_count = 0;        // total number of valid tasks

//RTOS 
int rtos_mode;             // mode
#define MODE_COOPERATIVE 0
#define MODE_PREEMPTIVE  1

// Stack Pointer
int *SP = (int*)0x1E;

//Scheduler
unsigned int Last_PID;       // last process created
unsigned int MAX_PRIORITY=0; // highest process priority

//Task Control Block
struct _tcb
{
	unsigned int state;               // see STATE_ values above
	unsigned int pid;                 // used to uniquely identify process
	unsigned int sp;                  // location of stack pointer for process
	unsigned int priority;            // 0=lowest, 7=highest
	unsigned int current_priority;
	unsigned int ticks;               // ticks until sleep complete
	unsigned int semaphore_value;	   // Address of semaphore, task is waiting on
	unsigned int semaphore_que_size;//que_size=1 if task is waiting on semaphore
	char strProcessName[10];
	unsigned int iSignalFlag;	    // =1 ,when a signal is received
	unsigned int pcount;              // priority + 1
	unsigned int SPLimit;
} tcb[MAX_TASKS];

// User defined stack
unsigned int stack[MAX_TASKS][256];

//-----------------------------------------------------------------------------
// RTOS Kernel functions                
//-----------------------------------------------------------------------------

// ISR of Timer 3
//Function Desc :: 1. Updated ticks for delayed processes
//				   2. For pre-emptive mode task switches every 1ms
void __attribute__((__interrupt__, no_auto_psv)) _T3Interrupt(void)
{
	int i=0;
	int *FP = (int*)0x1C;
	IFS0bits.T3IF = 0;
	if(rtos_mode == MODE_PREEMPTIVE)
	{
        //
		asm("ulnk");
		asm("POP WREG7");
		asm("POP WREG6");
		asm("POP WREG5");
		asm("POP WREG4");
		asm("POP WREG3");
		asm("POP WREG2");
		asm("POP WREG1");
		asm("POP WREG0");
		asm("POP RCOUNT");

		asm("lnk #0x4");

		asm("PUSH WREG14");
		asm("PUSH RCOUNT");
		asm("PUSH TBLPAG");
		asm("PUSH PSVPAG");
		asm("PUSH DCOUNT");
		asm("PUSH DOSTARTL");
		asm("PUSH DOSTARTH");
		asm("PUSH DOENDL");
		asm("PUSH DOENDH");
		asm("PUSH WREG0");
		asm("PUSH WREG1");
		asm("PUSH WREG2");
		asm("PUSH WREG3");
		asm("PUSH WREG4");
		asm("PUSH WREG5");
		asm("PUSH WREG6");
		asm("PUSH WREG7");
		asm("PUSH WREG8");
		asm("PUSH WREG9");
		asm("PUSH WREG10");
		asm("PUSH WREG11");
		asm("PUSH WREG12");
		asm("PUSH WREG13");
		tcb[task_current].sp = *SP;
	}
	//  Updating ticks for delayed processes
	for(i = 0; i < MAX_QUEUE_SIZE; i++)
	{
		if (tcb[i].state == STATE_DELAYED)
			if ((--tcb[i].ticks) == 0)
			{
				tcb[i].state=STATE_READY;
			}
	}
	// In Pre-Emptive mode task swtiches every 1 ms
	if(rtos_mode == MODE_PREEMPTIVE)
	{
		task_current = rtos_scheduler();
		SPLIM=tcb[task_current].SPLimit;
		*SP = tcb[task_current].sp;
		// continue the startup of the first process
		asm("POP WREG13");
		asm("POP WREG12");
		asm("POP WREG11");
		asm("POP WREG10");
		asm("POP WREG9");
		asm("POP WREG8");
		asm("POP WREG7");
		asm("POP WREG6");
		asm("POP WREG5");
		asm("POP WREG4");
		asm("POP WREG3");
		asm("POP WREG2");
		asm("POP WREG1");
		asm("POP WREG0");
		asm("POP DOENDH");
		asm("POP DOENDL");
		asm("POP DOSTARTH");
		asm("POP DOSTARTL");
		asm("POP DCOUNT");
		asm("POP PSVPAG");
		asm("POP TBLPAG");
		asm("POP RCOUNT");
		asm("POP WREG14");
		asm("ulnk");
		asm("retfie");
	}
}

// Function Desc:: Initializes all tasks to Invalid at start of RTOS
void rtos_init(int mode)
{
	int i;
	rtos_mode = mode;
	// no tasks running
	task_count = 0;
	// clear out tcb records
	for (i = 0; i < MAX_TASKS; i++)
	{
		tcb[i].state = STATE_INVALID;
		tcb[i].pid = 0;
	}
	init_timer3();
}

// Function Desc :: Priority based scheduler for assigning cpu time 
// for each task
int rtos_scheduler()
{
	// REQUIRED: Implement prioritization to 8 levels
	static int ok;
	static int task = 0xFF;
	static int cycle_count = 1;  // = MAX_PRIORITY + 1
	ok = FALSE;
	while (!ok)
	{
		task++;
		if (task >= MAX_TASKS)
			task = 0;

		if(cycle_count <= (MAX_PRIORITY + 1))
		{
			if(tcb[task].pcount == 0)
				ok = FALSE;
			else if(tcb[task].pcount != 0)
			{
				tcb[task].pcount--;
				ok = TRUE;
			}
			if(tcb[task].pid == Last_PID)
				cycle_count++;
		}
		else
		{
			cycle_count = 1;
			init_pcount();
		}
		if(ok)              // gives chance to all tasks
				ok = (tcb[task].state == STATE_READY);
	}
	return task;
}

// Function Desc :: Seed initial values into the stack to make
// it look like it already ran before
int create_process(_fn fn, int priority,char strName[10])
{
	int ok = FALSE;
	int i = 0;
	long (*fAddr)(long) = fn;
	int found = FALSE;
	IEC0bits.T3IE = 0;
	// save starting address if room in task list
	if (task_count < MAX_TASKS)
	{
		// make sure fn not already in list (prevent reentrancy)
		while (!found && (i < MAX_TASKS))
		{
			found = (tcb[i++].pid == (unsigned int) fn);
		}
		if (!found)
		{
			// find first available tcb record
			i = 0;
			while (tcb[i].state != STATE_INVALID) {i++;}
			// REQUIRED: seed the stack with initial values to look 
			// like task has already run
			tcb[i].state = STATE_READY;
			tcb[i].pid = (unsigned int) fn;
			// REQUIRED: set the original SP pointer location(depends on init above)
			stack[i][0] = (unsigned int)(fAddr)&0xFFFF;
			stack[i][1] = ((((unsigned int)fAddr)>>16))&0x007F;
			stack[i][2] = &stack[i][3]; // WREG14
			stack[i][3] = 0; //RCOUNT
			stack[i][4] = 0; //TBLPAG
			stack[i][5] = 0; //PSVPAG
			stack[i][6] = 0; //DCOUNT
			stack[i][7] = 0; //DOSTARTL
			stack[i][8] = 0; //DOSTARTH
			stack[i][9] = 0; //DOENDL
			stack[i][10] = 0; //DOENDH
			stack[i][11] = 0; //WREG0
			stack[i][12] = 1; //WREG1
			stack[i][13] = 2; //WREG2
			stack[i][14] = 3; //WREG3
			stack[i][15] = 4; //WREG4
			stack[i][16] = 5; //WREG5
			stack[i][17] = 6; //WREG6
			stack[i][18] = 7; //WREG7
			stack[i][19] = 8; //WREG8
			stack[i][20] = 9; //WREG9
			stack[i][21] = 10;//WREG10
			stack[i][22] = 11;//WREG11
			stack[i][23] = 12;//WREG12
			stack[i][24] = 13;//WREG13
			tcb[i].sp = (int)&stack[i][25]; // add offset as needed
			tcb[i].priority = priority;    
			tcb[i].current_priority = priority;    
			tcb[i].SPLimit=&stack[i][240];
			strcpy(tcb[i].strProcessName, strName);
			tcb[i].iSignalFlag = 0;

			Last_PID = (unsigned int) fn;
			if (priority >= MAX_PRIORITY)
				MAX_PRIORITY = priority;
			// increment task count
			task_count++;
			ok = TRUE;
		}
	}
	IEC0bits.T3IE = 1;
	return ok;
}
// REQUIRED: modify this function to destroy a process
// Function Desc :: Kills a task and removes the task from the semaphore queue
// the task has been waiting on
int destroy_process(_fn fn, int priority)
{
	int i, j, k;
	for(i = 0; i < MAX_TASKS; i++)
	{
		if(tcb[i].pid == (unsigned int )fn)
		{
			if(tcb[i].state == STATE_DELAYED)
				tcb[i].ticks = 0;
			else if(tcb[i].state == STATE_BLOCKED)
			{
				if(tcb[i].semaphore_que_size != 0 )
				{
					s = (struct semaphore*)tcb[i].semaphore_value;
					for(j = 0; j < s->queue_size; j++)
					{
						if(s->process_queue[j]== i)
						{
							for(k = j; k<s->queue_size; k++)
							{
								s->process_queue[k] = s->process_queue[k+1];
							}
							s->queue_size--;
						}
					}
					tcb[i].semaphore_que_size = 0;
				}
			}
			tcb[i].state=STATE_INVALID;
			strcpy(tcb[i].strProcessName,"");
			if(Last_PID == tcb[i].pid)
			{
				for(j = MAX_TASKS-1; j >= 0; j--)
				{
					if(tcb[j].state != STATE_INVALID)
					{
						Last_PID = tcb[j].pid;
						break;
					}
				}	
			}
			tcb[i].pid = 0;
			tcb[i].sp = 0;
			task_count--;
		}
	}
	return 0;
}

void rtos_start()
{
	int *SP = (int*)0x1E;
	IEC0bits.T3IE = 0; // Disable Timer3 interrupt
	task_current = rtos_scheduler();
	SPLIM=tcb[task_current].SPLimit;
	*SP = tcb[task_current].sp;
	// continue the startup of the first process
	asm("POP WREG13");
	asm("POP WREG12");
	asm("POP WREG11");
	asm("POP WREG10");
	asm("POP WREG9");
	asm("POP WREG8");
	asm("POP WREG7");
	asm("POP WREG6");
	asm("POP WREG5");
	asm("POP WREG4");
	asm("POP WREG3");
	asm("POP WREG2");
	asm("POP WREG1");
	asm("POP WREG0");
	asm("POP DOENDH");
	asm("POP DOENDL");
	asm("POP DOSTARTH");
	asm("POP DOSTARTL");
	asm("POP DCOUNT");
	asm("POP PSVPAG");
	asm("POP TBLPAG");
	asm("POP RCOUNT");
	asm("POP WREG14");
	asm("ulnk");
	IEC0bits.T3IE = 1; // Enable Timer3 interrupt
	asm("retfie");
}



// Function Desc :: Called by a task when it voluntarily relinquishes
// CPU time to another task to run after context saving
void yield()
{
	// push registers, call scheduler, pop registers, return to new function
	int *SP = (int*)0x1E;
	IEC0bits.T3IE = 0; // Disable Timer3 interrupt

	asm("PUSH WREG0");
	asm("PUSH WREG1");
	asm("PUSH WREG2");
	asm("PUSH WREG3");

	asm("MOV W15,W0");
	asm("SUB #0x0E,W0");  
	asm("MOV SR,W1");
	asm("AND #0x00FF,W1");
	asm("SL W1,#0x08,W1");
	asm("MOV CORCON,W2");
	asm("AND #0x0008,W2");
	asm("SL W2,#0x04,W2");
	asm("IOR W1,W2,W1");
	asm("MOV [W0],W2");
	asm("IOR W2,W1,W2");
	asm("MOV W2,[W0]");
	asm("ADD #0x0E,W0");
	asm("MOV W0,W15");

	asm("POP WREG3");
	asm("POP WREG2");
	asm("POP WREG1");
	asm("POP WREG0");
	// *temp=*temp|(((SR&0x00FF)<<8)|((CORCON&0x0008)<<4));

	asm("PUSH WREG14");
	asm("PUSH RCOUNT");
	asm("PUSH TBLPAG");
	asm("PUSH PSVPAG");
	asm("PUSH DCOUNT");
	asm("PUSH DOSTARTL");
	asm("PUSH DOSTARTH");
	asm("PUSH DOENDL");
	asm("PUSH DOENDH");
	asm("PUSH WREG0");
	asm("PUSH WREG1");
	asm("PUSH WREG2");
	asm("PUSH WREG3");
	asm("PUSH WREG4");
	asm("PUSH WREG5");
	asm("PUSH WREG6");
	asm("PUSH WREG7");
	asm("PUSH WREG8");
	asm("PUSH WREG9");
	asm("PUSH WREG10");
	asm("PUSH WREG11");
	asm("PUSH WREG12");
	asm("PUSH WREG13");
	tcb[task_current].sp = *SP;     
	task_current = rtos_scheduler();
	SPLIM=tcb[task_current].SPLimit;
	*SP = tcb[task_current].sp;
	asm("POP WREG13");
	asm("POP WREG12");
	asm("POP WREG11");
	asm("POP WREG10");
	asm("POP WREG9");
	asm("POP WREG8");
	asm("POP WREG7");
	asm("POP WREG6");
	asm("POP WREG5");
	asm("POP WREG4");
	asm("POP WREG3");
	asm("POP WREG2");
	asm("POP WREG1");
	asm("POP WREG0");
	asm("POP DOENDH");
	asm("POP DOENDL");
	asm("POP DOSTARTH");
	asm("POP DOSTARTL");
	asm("POP DCOUNT");
	asm("POP PSVPAG");
	asm("POP TBLPAG");
	asm("POP RCOUNT");
	asm("POP WREG14");
	asm("ulnk");
	IEC0bits.T3IE = 1; // Enable Timer3 interrupt
	asm("retfie");
}


// Function Desc :: Called by a task when it voluntarily relinquishes 
// CPU time to another task to run after context saving to run after 
// a certain amount of min time specified
void sleep(unsigned int tick)
{
	// push registers, set state to delayed, store timeout, call scheduler, 
	// pop registers, return to new function
	int *SP = (int*)0x1E;
	IEC0bits.T3IE = 0; // Disable Timer3 interrupt
	asm("PUSH WREG0");
	asm("PUSH WREG1");
	asm("PUSH WREG2");
	asm("PUSH WREG3");

	asm("MOV W15,W0");
	asm("SUB #0x0E,W0");

	asm("MOV SR,W1");
	asm("AND #0x00FF,W1");
	asm("SL W1,#0x08,W1");
	asm("MOV CORCON,W2");
	asm("AND #0x0008,W2");
	asm("SL W2,#0x04,W2");
	asm("IOR W1,W2,W1");
	asm("MOV [W0],W2");
	asm("IOR W2,W1,W2");
	asm("MOV W2,[W0]");
	asm("ADD #0x0E,W0");
	asm("MOV W0,W15");

	asm("POP WREG3");
	asm("POP WREG2");
	asm("POP WREG1");
	asm("POP WREG0");


	asm("PUSH WREG14");
	asm("PUSH RCOUNT");
	asm("PUSH TBLPAG");
	asm("PUSH PSVPAG");
	asm("PUSH DCOUNT");
	asm("PUSH DOSTARTL");
	asm("PUSH DOSTARTH");
	asm("PUSH DOENDL");
	asm("PUSH DOENDH");
	asm("PUSH WREG0");
	asm("PUSH WREG1");
	asm("PUSH WREG2");
	asm("PUSH WREG3");
	asm("PUSH WREG4");
	asm("PUSH WREG5");
	asm("PUSH WREG6");
	asm("PUSH WREG7");
	asm("PUSH WREG8");
	asm("PUSH WREG9");
	asm("PUSH WREG10");
	asm("PUSH WREG11");
	asm("PUSH WREG12");
	asm("PUSH WREG13");
	tcb[task_current].ticks = tick;
	tcb[task_current].state = STATE_DELAYED;
	tcb[task_current].sp = *SP;
	task_current = rtos_scheduler();
	SPLIM=tcb[task_current].SPLimit;
	*SP = tcb[task_current].sp;
	asm("POP WREG13");
	asm("POP WREG12");
	asm("POP WREG11");
	asm("POP WREG10");
	asm("POP WREG9");
	asm("POP WREG8");
	asm("POP WREG7");
	asm("POP WREG6");
	asm("POP WREG5");
	asm("POP WREG4");
	asm("POP WREG3");
	asm("POP WREG2");
	asm("POP WREG1");
	asm("POP WREG0");
	asm("POP DOENDH");
	asm("POP DOENDL");
	asm("POP DOSTARTH");
	asm("POP DOSTARTL");
	asm("POP DCOUNT");
	asm("POP PSVPAG");
	asm("POP TBLPAG");
	asm("POP RCOUNT");
	asm("POP WREG14");
	asm("ulnk");
	IEC0bits.T3IE = 1; // Enable Timer3 interrupt
	asm("retfie");
}

// return if avail, else yield to scheduler
// Function Desc :: Called by a task when it voluntarily relinquishes          
// CPU time to another task to run after context saving while waiitng 
// on a resource
void wait(void* p)
{
	int *SP = (int*)0x1E;
	IEC0bits.T3IE = 0; // Disable Timer3 interrupt
	asm("PUSH WREG0");
	asm("PUSH WREG1");
	asm("PUSH WREG2");
	asm("PUSH WREG3");

	asm("MOV W15,W0");
	asm("SUB #0x0E,W0");

	asm("MOV SR,W1");
	asm("AND #0x00FF,W1");
	asm("SL W1,#0x08,W1");
	asm("MOV CORCON,W2");
	asm("AND #0x0008,W2");
	asm("SL W2,#0x04,W2");
	asm("IOR W1,W2,W1");
	asm("MOV [W0],W2");
	asm("IOR W2,W1,W2");
	asm("MOV W2,[W0]");
	asm("ADD #0x0E,W0");
	asm("MOV W0,W15");

	asm("POP WREG3");
	asm("POP WREG2");
	asm("POP WREG1");
	asm("POP WREG0");


	asm("PUSH WREG14");
	asm("PUSH RCOUNT");
	asm("PUSH TBLPAG");
	asm("PUSH PSVPAG");
	asm("PUSH DCOUNT");
	asm("PUSH DOSTARTL");
	asm("PUSH DOSTARTH");
	asm("PUSH DOENDL");
	asm("PUSH DOENDH");
	asm("PUSH WREG0");
	asm("PUSH WREG1");
	asm("PUSH WREG2");
	asm("PUSH WREG3");
	asm("PUSH WREG4");
	asm("PUSH WREG5");
	asm("PUSH WREG6");
	asm("PUSH WREG7");
	asm("PUSH WREG8");
	asm("PUSH WREG9");
	asm("PUSH WREG10");
	asm("PUSH WREG11");
	asm("PUSH WREG12");
	asm("PUSH WREG13");
	tcb[task_current].sp = *SP;
	s=(struct semaphore*)p;
	if (s->count>0)
	{
		s->count--;
	}
	else
	{
		s->process_queue[s->queue_size]=task_current;
		s->queue_size++;
		tcb[task_current].state=STATE_BLOCKED;
		tcb[task_current].semaphore_que_size++;
		tcb[task_current].semaphore_value=p;
		task_current=rtos_scheduler();
		SPLIM=tcb[task_current].SPLimit;
		*SP=tcb[task_current].sp;
	}
	asm("POP WREG13");
	asm("POP WREG12");
	asm("POP WREG11");
	asm("POP WREG10");
	asm("POP WREG9");
	asm("POP WREG8");
	asm("POP WREG7");
	asm("POP WREG6");
	asm("POP WREG5");
	asm("POP WREG4");
	asm("POP WREG3");
	asm("POP WREG2");
	asm("POP WREG1");
	asm("POP WREG0");
	asm("POP DOENDH");
	asm("POP DOENDL");
	asm("POP DOSTARTH");
	asm("POP DOSTARTL");
	asm("POP DCOUNT");
	asm("POP PSVPAG");
	asm("POP TBLPAG");
	asm("POP RCOUNT");
	asm("POP WREG14");
	asm("ulnk");
	IEC0bits.T3IE = 1; // Enable Timer3 interrupt
	asm("retfie");
}

// Function Desc :: Called by a task when it needs to signal the availabiliy
// of a resource
void signal(void* p)
{
	int task_index;
	int i;
	s =(struct semaphore*) p;
	s->count++;

	if(s->queue_size != 0)
	{

		task_index = s->process_queue[0];
		for(i = 0; i < (s->queue_size); ++i )
		{
			s->process_queue[i] = s->process_queue[i+1];
		}
		s->queue_size--;
		tcb[task_index].iSignalFlag=1;
		tcb[task_index].state = STATE_READY;
	}
}
//UART 1 RX ISR
// Function Desc :: Signals the availaility of a character
void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void)
{
	IFS0bits.U1RXIF = 0;
	// clear out any overflow error condition
	if (U1STAbits.OERR == 1)
		U1STAbits.OERR = 0;
	//rx_232_que[write_232_index]=U1RXREG;
	//write_232_index++;
	signal(&rx_232_dataSemaphore);
}
//-----------------------------------------------------------------------------
// Subroutines                
//-----------------------------------------------------------------------------

// Function Desc :: Used to initialize pcount for each task to
// max number of runs in a cycle
void init_pcount()
{
	int k;
	for(k = 0; k < MAX_TASKS; ++k)
	{
		if(tcb[k].state != STATE_INVALID)
			tcb[k].pcount = tcb[k].priority+1;
	}
}
// Function Desc :: To initialise a semaphore with initial values
void init(void* p, int count)
{
	s = p;
	s->count = count;
	s->queue_size = 0;
	semaphore_count++;
}

// Initialize timer 3 to run every 1 ms.
void init_timer3()
{
	// Timer 3
	T3CONbits.TON = 0;   // Disable Timer
	T3CONbits.TCS = 0;   // Select internal instruction cycle clock
	T3CONbits.TGATE = 0; // Disable Gated Timer mode
	T3CONbits.TCKPS = 3; // Select 1:256 Prescaler
	TMR3 = 0x00;         // Clear timer register
	PR3 = 156;           // Load the period value
	IFS0bits.T3IF = 0;   // Clear Timer3 Interrupt Flag
	IEC0bits.T3IE = 0;   // Disable Timer3 interrupt
	T3CONbits.TON = 1;   // Start Timer
	IEC0bits.T3IE = 1;   // Enable Timer3
}
// Initialize UART
void serial_init()
{
	// set baud rate
	U1BRG = BAUD_38400;
	// enable uarts, 8N1, low speed brg
	U1MODE = 0x8000;
	// enable tx and rx
	U1STA = 0x0440;
	IFS0bits.U1TXIF=0;
	IEC0bits.U1TXIE=0;
	//U1STA.URXISEL=1;
	IFS0bits.U1RXIF=0;
	IEC0bits.U1RXIE=1;
}



// Initialize Hardware
void init_hw()
{
	AD1PCFGLbits.PCFG4 = 1;                    // make selected pins digital
	AD1PCFGLbits.PCFG5 = 1;
	LATBbits.LATB2 = 0;                        // write 0 into output latches
	LATBbits.LATB3 = 0;
	LATBbits.LATB4 = 0;
	LATBbits.LATB5 = 0;
	TRISBbits.TRISB2 = 0;                      // led pins outputs
	TRISBbits.TRISB3 = 0;
	TRISBbits.TRISB4 = 0;
	TRISBbits.TRISB5 = 0;
	CNPU1bits.CN11PUE = 1;                   // enable pull-ups for push buttons
	CNPU1bits.CN12PUE = 1;
	CNPU1bits.CN13PUE = 1;
	CNPU1bits.CN14PUE = 1;
	RPINR18bits.U1RXR = 11;                    // assign U1RX to RP11
	RPOR5bits.RP10R = 3;                       // assign U1TX to RP10

	PLLFBDbits.PLLDIV = 38;                    // pll feedback divider = 40;
	CLKDIVbits.PLLPRE = 0;                     // pll pre divider = 2
	CLKDIVbits.PLLPOST = 0;                    // pll post divider = 2
}


// Function Desc ::returns the value fo the button pressed
int read_pbs()
{
	return (~PORTB >> 12);
}

// ---------------------------------------------------------------------------
//  Task functions
// ---------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose

void idle()
{
	while(TRUE) 
	{ 
		PIN_ORANGE = 1;
		__delay_ms(1);
		PIN_ORANGE = 0;
		yield();
	}
}

void flash_4hz()
{
	while(TRUE)
	{
		PIN_GREEN ^= 1;
		sleep(125);
	}
}

void one_shot()
{
	while(TRUE)
	{
		//	printf("flash count - %d\n",s->count);
		wait(&flash_req);
		if(tcb[task_current].iSignalFlag == 1){
			tcb[task_current].iSignalFlag=0;
			continue;
		} 
		PIN_YELLOW = 1;
		sleep(1000);
		PIN_YELLOW = 0;
	}
}

void part_of_lengthy_fn()
{
	// represent some lengthy operation
	__delay_ms(1);
	// give another process a chance
	yield();
}

void lengthy_fn()
{
	long i;
	while(TRUE)
	{
		for (i = 0; i < 4000; i++)
		{
			part_of_lengthy_fn();
		}
		PIN_RED ^= 1;
	}
}

void read_keys()
{
	int buttons;
	while(TRUE)
	{
		wait(&key_released);
		buttons = 0;
		while (buttons == 0)
		{

			buttons = read_pbs();
			yield();
		}
		signal(&key_pressed);
		if ((buttons & 1) != 0)
		{
			sleep(115);
			PIN_YELLOW ^= 1;
			PIN_RED = 1;
		}
		if ((buttons & 2) != 0)
		{
			sleep(115);
			signal(&flash_req);
			PIN_RED = 0;
		}
		if ((buttons & 4) != 0)
		{
			sleep(115);
			create_process(flash_4hz,7,"flash_4hz");
		}
		if ((buttons & 8) != 0)
		{
			destroy_process(flash_4hz, 0);
		}

		yield();
	}
}

void debounce()
{
	int count;
	while(TRUE)
	{
		wait(&key_pressed);
		count = 10;
		while (count != 0)
		{  
			sleep(10);
			if (read_pbs() == 0)
				count--;
			else
				count = 10;
		}
		signal(&key_released);
	}
}

void uncooperative()
{
	while(TRUE)
	{
		while (read_pbs() == 8)
		{
		}
		yield();
	}
}

// Function Desc :: Prints the task details on the console for every 3 sec
void dump_Data()
{
	static int i=0;
	unsigned int k,j=0;
	char strState[15],strSemName[10];

	//Clear the screen and goto 0,0
	printf("\033[2J");
	printf("\033[0;0f");

	while(1)
	{
		for(i=0;i<MAX_TASKS;i++)
		{
			if(i==0)
			{
				//save cursor position
				printf("\033[s");
				//Force cursor to the Home Position
				printf("\033[0;0f");
				printf("\nIndex\t   TaskName\t    PID\t\t STATE\t\t   SP\t\n");
			}

			switch(tcb[i].state)
			{
				case 0:	strcpy(strState,"INVALID    ");break;
				case 1:	strcpy(strState,"READY    ");break;
				case 2:	if(tcb[i].semaphore_que_size != 0)
						{
							k=tcb[i].semaphore_value;
							if (k==&flash_req)
								strcpy(strSemName,"flash_req");
							else if(k==&key_pressed)
								strcpy(strSemName,"Key_Press");
							else if(k==&key_released)
								strcpy(strSemName,"Key_Rel");
							else if(k==&rx_232_dataSemaphore)
								strcpy(strSemName,"Rx_232_Dat");
						}	
						strcpy(strState,"  BLOCKED @ ");
						strcat(strState,strSemName);
						break;
				case 3:	sprintf(strState,"DELAYED(%5d)   ",tcb[i].ticks);break;
			}
			printf("\n\033[J%2d\t%10s\t%7d\t%16s\t%6x",i,tcb[i].strProcessName,
					tcb[i].pid,strState,tcb[i].sp);
			if(i==(MAX_TASKS-1))
			{
				printf("\033[u");
				if(j ==0)
				{
					printf("Enter Command :: ");
					printf("\033[0;17f");
					j=1;
				}	
			}
			sleep(25);	
		}
		sleep(3000);
	}		
}	
// Function Desc :: Upon pressing enter in the console, i prints the 
//                  text entered
void doAction()
{
	PIN_RED^=1;
	printf("\033[20;0f");
	printf("%s\033[J",rx_232_data);
	printf("\033[0;70f\033[1J\033[0;0fEnter Command ::");

}	

void rx_232_dataProcess()
{
	unsigned char strRx;
	while(1)
	{
		wait(&rx_232_dataSemaphore);
		if(tcb[task_current].iSignalFlag == 1)
		{
			tcb[task_current].iSignalFlag = 0;
			continue;
		}	
		strRx=U1RXREG;
		//backspace
		if(strRx == 8)
		{
			if(--write_232_index <0)
				write_232_index=0;
			printf("\033[k");
		}
		else if(strRx == 13 || strRx == 10)
		{
			rx_232_data[write_232_index]='\0';
			if(write_232_index != 0)
				doAction();
			else
				printf("\033[0;17f");
			write_232_index=0;
		}
		else
		{
			if(write_232_index<MAX_232_Que)
				rx_232_data[write_232_index++]=strRx;	
		}		
	}	

}	

// REQUIRED: add an isr that signals the availability of serial data

// REQUIRED:add a process that waits for serial data then takes some action

// REQUIRED: add a process to print the tcb information (PID, current SP,
// current PC for all tasks in the TCB every 3 seconds

//-----------------------------------------------------------------------------
// Main                
//-----------------------------------------------------------------------------

int main(void)
{
	int ok;
	int pb;

	// initialize hardware
	init_hw();  

	// power-up flash
	PIN_RED = 1;
	__delay32(10000000);
	PIN_RED = 0;
	__delay32(10000000);

	// init semaphores
	init(&key_pressed, 0);
	init(&key_released, 1);
	init(&flash_req, 5);
	init(&rx_232_data, 5);	
	serial_init();
	//init_timer1();

	// initialize selected RTOS
	ok = FALSE;

	while (!ok)
	{
		pb = read_pbs();
		if (pb & 4) 
		{
			ok = TRUE;
			__delay_ms(100);
			rtos_init(MODE_COOPERATIVE);
		}
		if (pb & 8) 
		{
			ok = TRUE;
			__delay_ms(100);
			rtos_init(MODE_PREEMPTIVE);
		}
	}

	ok =  create_process(idle, 0,"idle") >= 0;
	//ok &= create_process(flash_4hz, 7,"flash_4hz") >= 0;
	// add other processes
	ok &= create_process(lengthy_fn, 1,"lengthy_fn") >= 0;
	ok &= create_process(one_shot, 4,"one_shot") >= 0;
	ok &= create_process(read_keys, 6,"read_keys") >= 0;
	ok &= create_process(debounce, 4,"debounce") >= 0;
	ok &= create_process(uncooperative, 2,"Uncooperative") >= 0;
	ok &= create_process(dump_Data,3,"DumpData") >=0;
	ok &= create_process(rx_232_dataProcess,3,"Rx_232_Data") >=0;
	init_pcount();

	// start up rtos
	if (ok) 
		rtos_start(); // never returns
	else
		PIN_RED = 1;

	return 0;
	// don't delete this unreachable code
	// if a function is only called once in your code, it will be
	// accessed with two goto instructions instead of call-return,
	// so any stack-based code will not function correctly
	yield(); sleep(0); wait(0); signal(0);
}
