/*
Copyright 2018 Embedded Microprocessor Benchmark Consortium (EEMBC)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

Original Author: Shay Gal-on
*/

#include <sys/platform.h>
#include <sys/adi_core.h>
#include <runtime/int/interrupt.h>
#include <cycle_count.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "adi_initialize.h"
#include "coremark.h"
#if CALLGRIND_RUN
#include <valgrind/callgrind.h>
#endif


#if (MEM_METHOD==MEM_MALLOC)
//#include <malloc.h>
/* Function: portable_malloc
	Provide malloc() functionality in a platform specific way.
*/
void *portable_malloc(size_t size) {
	return malloc(size);
}
/* Function: portable_free
	Provide free() functionality in a platform specific way.
*/
void portable_free(void *p) {
	free(p);
}
#else
void *portable_malloc(size_t size) {
	return NULL;
}
void portable_free(void *p) {
	p=NULL;
}
#endif

#if (SEED_METHOD==SEED_VOLATILE)
#if VALIDATION_RUN
	volatile ee_s32 seed1_volatile=0x3415;
	volatile ee_s32 seed2_volatile=0x3415;
	volatile ee_s32 seed3_volatile=0x66;
#endif
#if PERFORMANCE_RUN
	volatile ee_s32 seed1_volatile=0x0;
	volatile ee_s32 seed2_volatile=0x0;
	volatile ee_s32 seed3_volatile=0x66;
#endif
#if PROFILE_RUN
	volatile ee_s32 seed1_volatile=0x8;
	volatile ee_s32 seed2_volatile=0x8;
	volatile ee_s32 seed3_volatile=0x8;
#endif
	volatile ee_s32 seed4_volatile=0;
	volatile ee_s32 seed5_volatile=0;
#endif
/* Porting: Timing functions
	How to capture time and convert to seconds must be ported to whatever is supported by the platform.
	e.g. Read value from on board RTC, read value from cpu clock cycles performance counter etc. 
	Sample implementation for standard time.h and windows.h definitions included.
*/
/* Define: TIMER_RES_DIVIDER
	Divider to trade off timer resolution and total time that can be measured.

	Use lower values to increase resolution, but make sure that overflow does not occur.
	If there are issues with the return value overflowing, increase this value.
	*/
#if USE_CLOCK
	#define NSECS_PER_SEC CLOCKS_PER_SEC
	#define EE_TIMER_TICKER_RATE 1000
	#define CORETIMETYPE clock_t 
	#define GETMYTIME(_t) (*_t=clock())
	#define MYTIMEDIFF(fin,ini) ((fin)-(ini))
	#define TIMER_RES_DIVIDER 1
	#define SAMPLE_TIME_IMPLEMENTATION 1
#elif defined(_MSC_VER)
	#define NSECS_PER_SEC 10000000
	#define EE_TIMER_TICKER_RATE 1000
	#define CORETIMETYPE FILETIME
	#define GETMYTIME(_t) GetSystemTimeAsFileTime(_t)
	#define MYTIMEDIFF(fin,ini) (((*(__int64*)&fin)-(*(__int64*)&ini))/TIMER_RES_DIVIDER)
	/* setting to millisces resolution by default with MSDEV */
	#ifndef TIMER_RES_DIVIDER
	#define TIMER_RES_DIVIDER 1000
	#endif
	#define SAMPLE_TIME_IMPLEMENTATION 1
#elif HAS_TIME_H
	#define NSECS_PER_SEC 1000000000
	#define EE_TIMER_TICKER_RATE 1000
	#define CORETIMETYPE struct timespec 
	#define GETMYTIME(_t) clock_gettime(CLOCK_REALTIME,_t)
	#define MYTIMEDIFF(fin,ini) ((fin.tv_sec-ini.tv_sec)*(NSECS_PER_SEC/TIMER_RES_DIVIDER)+(fin.tv_nsec-ini.tv_nsec)/TIMER_RES_DIVIDER)
	/* setting to 1/1000 of a second resolution by default with linux */
	#ifndef TIMER_RES_DIVIDER
	#define TIMER_RES_DIVIDER 1000000
	#endif
	#define SAMPLE_TIME_IMPLEMENTATION 1
#else
/*	#define NSECS_PER_SEC 1000000000
	#define EE_TIMER_TICKER_RATE 1
	#define CORETIMETYPE time_t
	#define GETMYTIME(_t) (*_t = time(NULL))
	#define MYTIMEDIFF(fin,ini) ((fin - ini)*(NSECS_PER_SEC/TIMER_RES_DIVIDER))
	#ifndef TIMER_RES_DIVIDER
	#define TIMER_RES_DIVIDER 1000000000
	#endif
	#define SAMPLE_TIME_IMPLEMENTATION 1*/
/*	#define NSECS_PER_SEC 1000000000
	#define EE_TIMER_TICKER_RATE 1000
	#define CORETIMETYPE ee_u32
	#define GETMYTIME(_t) (*_t = SDL_GetTicks())
	#define MYTIMEDIFF(fin,ini) ((fin - ini)*(NSECS_PER_SEC/TIMER_RES_DIVIDER))
	#ifndef TIMER_RES_DIVIDER
	#define TIMER_RES_DIVIDER 1000000
	#endif
	#define SAMPLE_TIME_IMPLEMENTATION 1*/
#endif
//#define EE_TICKS_PER_SEC (NSECS_PER_SEC / TIMER_RES_DIVIDER)
#define EE_TICKS_PER_SEC (450000000)

#if SAMPLE_TIME_IMPLEMENTATION
/** Define Host specific (POSIX), or target specific global time variables. */
static CORETIMETYPE start_time_val, stop_time_val;

/* Function: start_time
	This function will be called right before starting the timed portion of the benchmark.

	Implementation may be capturing a system timer (as implemented in the example code) 
	or zeroing some system parameters - e.g. setting the cpu clocks cycles to 0.
*/
void start_time(void) {
	GETMYTIME(&start_time_val );      
#if CALLGRIND_RUN
	CALLGRIND_START_INSTRUMENTATION
#endif
#if MICA
    asm volatile("int3");/*1 */
#endif
}
/* Function: stop_time
	This function will be called right after ending the timed portion of the benchmark.

	Implementation may be capturing a system timer (as implemented in the example code) 
	or other system parameters - e.g. reading the current value of cpu cycles counter.
*/
void stop_time(void) {
#if CALLGRIND_RUN
	 CALLGRIND_STOP_INSTRUMENTATION 
#endif
#if MICA
    asm volatile("int3");/*1 */
#endif
	GETMYTIME(&stop_time_val );      
}
/* Function: get_time
	Return an abstract "ticks" number that signifies time on the system.
	
	Actual value returned may be cpu cycles, milliseconds or any other value,
	as long as it can be converted to seconds by <time_in_secs>.
	This methodology is taken to accomodate any hardware or simulated platform.
	The sample implementation returns millisecs by default, 
	and the resolution is controlled by <TIMER_RES_DIVIDER>
*/
CORE_TICKS get_time(void) {
	CORE_TICKS elapsed=(CORE_TICKS)(MYTIMEDIFF(stop_time_val, start_time_val));
	return elapsed;
}
/* Function: time_in_secs
	Convert the value returned by get_time to seconds.

	The <secs_ret> type is used to accomodate systems with no support for floating point.
	Default implementation implemented by the EE_TICKS_PER_SEC macro above.
*/
secs_ret time_in_secs(CORE_TICKS ticks) {
	secs_ret retval=((secs_ret)ticks) / (secs_ret)EE_TICKS_PER_SEC;
	return retval;
}
#else 

static volatile uint32_t ticks;

/* The constant ADI_CFG_GP_TMR_NUM should probably be defined in FreeRTOSConfig.h */
#ifndef ADI_CFG_GP_TMR_NUM
#define ADI_CFG_GP_TMR_NUM 7
#endif

#if !defined (__ADSPSC589_FAMILY__) && !defined (__ADSPSC573_FAMILY__)
#error "This processor family is not supported"
#endif

#if !defined (ADI_CFG_GP_TMR_NUM)
#error "Set up tick management timer is enabled but no GP timer is specified"
#endif

#if (ADI_CFG_GP_TMR_NUM >= PARAM_TIMER0_NUMTIMERS)
#error "Incorrect GP Timer number specified for the processor."
#endif

/* Macros to construct the desired register name from the timer number. */
#define  REG_NAME_STR(RN, END)  pREG_TIMER0_TMR##RN##_##END
#define  REG_NAME(RN, END)      REG_NAME_STR(RN, END)

/* Macros to construct the desired interrupt name from the timer number. */
#define  INT_NAME_STR(RN)       INTR_TIMER0_TMR##RN
#define  INT_NAME(RN)           INT_NAME_STR(RN)

#define TIMER_NUM (ADI_CFG_GP_TMR_NUM)

#define pTIMERCFG REG_NAME(ADI_CFG_GP_TMR_NUM, CFG)
#define pTIMERCNT REG_NAME(ADI_CFG_GP_TMR_NUM, CNT)
#define pTIMERPER REG_NAME(ADI_CFG_GP_TMR_NUM, PER)
#define pTIMERWID REG_NAME(ADI_CFG_GP_TMR_NUM, WID)
#define pTIMERDLY REG_NAME(ADI_CFG_GP_TMR_NUM, DLY)
#define TIMERINT (uint32_t)INT_NAME(ADI_CFG_GP_TMR_NUM)

#define SET_TIMER_MSK_BIT ((uint16_t)(1u << TIMER_NUM))
#define CLR_TIMER_MSK_BIT ((uint16_t)~(1u << TIMER_NUM))

/* Handler for GP Timer interrupt. */
static void tickHandler(uint32_t id, void *param)
{
    ticks ++;

    /* Clear the timer interrupt latch before returning (write-1-to-clear) */
    *pREG_TIMER0_DATA_ILAT = SET_TIMER_MSK_BIT;
}

/*
 * Set up a GP Timer as the RTOS tick interrupt source.
 *
 * This function configures a GP Timer to be used for FreeRTOS timing services.
 * The GP Timer to be used is selected by a ADI_CFG_GP_TMR_NUM macro, which is
 * defined by the FreeRTOS UI.  This function only modifies the registers, and
 * parts of registers, that apply to the given GP timer.
 */

#define configCPU_CLOCK_HZ                      450000000
#define configSC5xx_CGU0_SSYSEL_DIVISOR         2
#define configSC5xx_CGU0_S0SEL_DIVISOR          2
#define configTICK_RATE_HZ                      10

/* Enable the GIC interrupt. */
static void EnableGICTimerInt(uint32_t id)
{
    /* Point to the base of the enable registers,
     * each enable register refers to 32 interrupts.
     * Interrupt IDs start at 32 for the Cortex A5, so bit 1
     * of the pREG_GICDST0_SPI_EN_SET0 applies to interrupt 32. */
    volatile uint32_t *pSetReg = pREG_GICDST0_SPI_EN_SET0;

    /* The following rotates to divide by 32. On Cortex-A the
     * result will always be at least 1 (IDs start at 32) so we
     * need to take that into account when indexing later.
     */
    const uint32_t nRegIndex  = id >> 5ul;      /* Shift to divide by 32. */

    /* Extract the bit position from the lower 5 bits of the
     * interrupt id. */
    const uint32_t nBitNum    = id & 0x1Ful;

    /* Subtract by 1 to get the correct GIC register.
     * IDs will always be 32 or higher, so the above arithmetic
     * will always yield at least 1.  The values passed to this
     * function will always be valid interrupt IDs. */
    pSetReg[nRegIndex - 1] = (1ul << nBitNum);
}


void vConfigureTickInterrupt(void)
{
	ticks = 0;

    /* GP timers are clocked from SCLK0 */
    const uint32_t nCycles =
        configCPU_CLOCK_HZ /
        configSC5xx_CGU0_SSYSEL_DIVISOR /
        configSC5xx_CGU0_S0SEL_DIVISOR /
        configTICK_RATE_HZ;

    /* Write 1 to configure the timer to "abrupt halt" configuration (doesn't stop timer) */
    *pREG_TIMER0_STOP_CFG_SET    =  SET_TIMER_MSK_BIT;

    /* Write 1 to set the timer to stop (does stop timer). */
    *pREG_TIMER0_RUN_CLR         =  SET_TIMER_MSK_BIT;

    /* Clear any interrupts (write 1 to clear) */
    *pREG_TIMER0_DATA_ILAT       =  SET_TIMER_MSK_BIT;
    *pREG_TIMER0_STAT_ILAT       =  SET_TIMER_MSK_BIT;

    /* Hide all the interrupts (write 1 to disable interrupt)*/
    *pREG_TIMER0_DATA_IMSK       |=  SET_TIMER_MSK_BIT;
    *pREG_TIMER0_STAT_IMSK       |=  SET_TIMER_MSK_BIT;

    /* Clear the trigger output mask (1 to disable) */
    *pREG_TIMER0_TRG_MSK         |=  SET_TIMER_MSK_BIT;

    /* Clear the trigger input mask (0 to disable) */
    *pREG_TIMER0_TRG_IE          &=  CLR_TIMER_MSK_BIT;

    /* Clear the trigger input and output masks */
    *pREG_TIMER0_TRG_MSK         &=  CLR_TIMER_MSK_BIT;
    *pREG_TIMER0_TRG_IE          &=  CLR_TIMER_MSK_BIT;

    adi_rtl_register_dispatched_handler(TIMERINT,   /* GP timer        */
                                        tickHandler,  /* Timer handler   */
                                        NULL);      /* No callback arg */

    EnableGICTimerInt(TIMERINT);

    /* Clear all registers before starting */
    *pTIMERCFG = 0u;
    *pTIMERDLY = 0u;
    *pTIMERWID = 0u;
    *pTIMERPER = 0u;

    /* Configure the individual timer */
    *pTIMERCFG = (uint16_t)(  ENUM_TIMER_TMR_CFG_CLKSEL_SCLK   /* Use the system clock as a  source */
                            | ENUM_TIMER_TMR_CFG_IRQMODE1      /* Generate pulse when Delay is over */
                            | ENUM_TIMER_TMR_CFG_PWMCONT_MODE  /* Continuous PWM mode */
                            | ENUM_TIMER_TMR_CFG_POS_EDGE      /* Interrupt on the positive pulse edge */
                            | ENUM_TIMER_TMR_CFG_EMU_NOCNT);   /* Timer stops during emulation */

    /* Set the timer delay */
#if 0
    *pTIMERDLY = nCycles;          /* Interrupt generated at this count */
    *pTIMERWID = 0x1u;             /* Pulse held high for 1 cycle       */
    *pTIMERPER = nCycles + 0x1u;   /* Sequence restarts after these cycles       */
#else
    *pTIMERDLY = nCycles - 0x1u;   /* Interrupt generated at this count */
    *pTIMERWID = 0x1u;             /* Pulse held high for 1 cycle       */
    *pTIMERPER = nCycles;          /* Sequence restarts after these cycles       */
#endif

    /* Enable data interrupts (0 for enable, 1 for disable) */
    *pREG_TIMER0_DATA_IMSK   &=  CLR_TIMER_MSK_BIT;

    /* Start the timer */
    *pREG_TIMER0_RUN_SET = SET_TIMER_MSK_BIT;
}

static uint32_t start_tick, end_tick;

/* Function: start_time
	This function will be called right before starting the timed portion of the benchmark.

	Implementation may be capturing a system timer (as implemented in the example code)
	or zeroing some system parameters - e.g. setting the cpu clocks cycles to 0.
*/
void start_time(void) {
	start_tick = ticks;
}
/* Function: stop_time
	This function will be called right after ending the timed portion of the benchmark.

	Implementation may be capturing a system timer (as implemented in the example code)
	or other system parameters - e.g. reading the current value of cpu cycles counter.
*/
void stop_time(void) {
	end_tick = ticks;
}
/* Function: get_time
	Return an abstract "ticks" number that signifies time on the system.

	Actual value returned may be cpu cycles, milliseconds or any other value,
	as long as it can be converted to seconds by <time_in_secs>.
	This methodology is taken to accomodate any hardware or simulated platform.
	The sample implementation returns millisecs by default,
	and the resolution is controlled by <TIMER_RES_DIVIDER>
*/
CORE_TICKS get_time(void) {
	CORE_TICKS elapsed = end_tick - start_tick;
	return elapsed;
}
/* Function: time_in_secs
	Convert the value returned by get_time to seconds.

	The <secs_ret> type is used to accomodate systems with no support for floating point.
	Default implementation implemented by the EE_TICKS_PER_SEC macro above.
*/
secs_ret time_in_secs(CORE_TICKS ticks) {
	secs_ret retval=((secs_ret)ticks) / (secs_ret)configTICK_RATE_HZ;
	return retval;
}
#endif /* SAMPLE_TIME_IMPLEMENTATION */

ee_u32 default_num_contexts=MULTITHREAD;

/* Function: portable_init
	Target specific initialization code 
	Test for some common mistakes.
*/
void portable_init(core_portable *p, int *argc, char *argv[])
{
	adi_initComponents();

	printf("Coremark for SC589-ARM\n");

#if PRINT_ARGS
	int i;
	for (i=0; i<*argc; i++) {
		printf("Arg[%d]=%s\n",i,argv[i]);
	}
#endif
	if (sizeof(ee_ptr_int) != sizeof(ee_u8 *)) {
		ee_printf("ERROR! Please define ee_ptr_int to a type that holds a pointer!\n");
	}
	if (sizeof(ee_u32) != 4) {
		ee_printf("ERROR! Please define ee_u32 to a 32b unsigned type!\n");
	}
#if (MAIN_HAS_NOARGC && (SEED_METHOD==SEED_ARG))
	ee_printf("ERROR! Main has no argc, but SEED_METHOD defined to SEED_ARG!\n");
#endif
	
#if (MULTITHREAD>1) && (SEED_METHOD==SEED_ARG)
	{
		int nargs=*argc,i;
		if ((nargs>1) && (*argv[1]=='M')) {
			default_num_contexts=parseval(argv[1]+1);
			if (default_num_contexts>MULTITHREAD)
				default_num_contexts=MULTITHREAD;
			/* Shift args since first arg is directed to the portable part and not to coremark main */
			--nargs;
			for (i=1; i<nargs; i++)
				argv[i]=argv[i+1];
			*argc=nargs;
		}
	}
#endif /* sample of potential platform specific init via command line, reset the number of contexts being used if first argument is M<n>*/

	vConfigureTickInterrupt();

	p->portable_id=1;
}
/* Function: portable_fini
	Target specific final code 
*/
void portable_fini(core_portable *p)
{
	p->portable_id=0;
}

#if (MULTITHREAD>1)

/* Function: core_start_parallel
	Start benchmarking in a parallel context.
	
	Three implementations are provided, one using pthreads, one using fork and shared mem, and one using fork and sockets.
	Other implementations using MCAPI or other standards can easily be devised.
*/
/* Function: core_stop_parallel
	Stop a parallel context execution of coremark, and gather the results.
	
	Three implementations are provided, one using pthreads, one using fork and shared mem, and one using fork and sockets.
	Other implementations using MCAPI or other standards can easily be devised.
*/
#if USE_PTHREAD
ee_u8 core_start_parallel(core_results *res) {
	return (ee_u8)pthread_create(&(res->port.thread),NULL,iterate,(void *)res);
}
ee_u8 core_stop_parallel(core_results *res) {
	void *retval;
	return (ee_u8)pthread_join(res->port.thread,&retval);
}
#elif USE_FORK
static int key_id=0;
ee_u8 core_start_parallel(core_results *res) {
	key_t key=4321+key_id;
	key_id++;
	res->port.pid=fork();
	res->port.shmid=shmget(key, 8, IPC_CREAT | 0666);
	if (res->port.shmid<0) {
		ee_printf("ERROR in shmget!\n");
	}
	if (res->port.pid==0) {
		iterate(res);
		res->port.shm=shmat(res->port.shmid, NULL, 0);
		/* copy the validation values to the shared memory area  and quit*/
		if (res->port.shm == (char *) -1) {
			ee_printf("ERROR in child shmat!\n");
		} else {
			memcpy(res->port.shm,&(res->crc),8);
			shmdt(res->port.shm);
		}
		exit(0);
	}
	return 1;
}
ee_u8 core_stop_parallel(core_results *res) {
	int status;
	pid_t wpid = waitpid(res->port.pid,&status,WUNTRACED);
	if (wpid != res->port.pid) {
		ee_printf("ERROR waiting for child.\n");
		if (errno == ECHILD) ee_printf("errno=No such child %d\n",res->port.pid);
		if (errno == EINTR) ee_printf("errno=Interrupted\n");
		return 0;
	}
	/* after process is done, get the values from the shared memory area */
	res->port.shm=shmat(res->port.shmid, NULL, 0);
	if (res->port.shm == (char *) -1) {
		ee_printf("ERROR in parent shmat!\n");
		return 0;
	} 
	memcpy(&(res->crc),res->port.shm,8);
	shmdt(res->port.shm);
	return 1;
}
#elif USE_SOCKET
static int key_id=0;
ee_u8 core_start_parallel(core_results *res) {
	int bound, buffer_length=8;
	res->port.sa.sin_family = AF_INET;
	res->port.sa.sin_addr.s_addr = htonl(0x7F000001);
	res->port.sa.sin_port = htons(7654+key_id);
	key_id++;
	res->port.pid=fork();
	if (res->port.pid==0) { /* benchmark child */
		iterate(res);
		res->port.sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
		if (-1 == res->port.sock) /* if socket failed to initialize, exit */   {
			ee_printf("Error Creating Socket");
		} else {
			int bytes_sent = sendto(res->port.sock, &(res->crc), buffer_length, 0,(struct sockaddr*)&(res->port.sa), sizeof (struct sockaddr_in));
			if (bytes_sent < 0)
				ee_printf("Error sending packet: %s\n", strerror(errno));
			close(res->port.sock); /* close the socket */
		}
		exit(0);
	} 
	/* parent process, open the socket */
	res->port.sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	bound = bind(res->port.sock,(struct sockaddr*)&(res->port.sa), sizeof(struct sockaddr));
	if (bound < 0)
		ee_printf("bind(): %s\n",strerror(errno));
	return 1;
}
ee_u8 core_stop_parallel(core_results *res) {
	int status;
	int fromlen=sizeof(struct sockaddr);
	int recsize = recvfrom(res->port.sock, &(res->crc), 8, 0, (struct sockaddr*)&(res->port.sa), &fromlen);
	if (recsize < 0) {
		ee_printf("Error in receive: %s\n", strerror(errno));
		return 0;
	}
	pid_t wpid = waitpid(res->port.pid,&status,WUNTRACED);
	if (wpid != res->port.pid) {
		ee_printf("ERROR waiting for child.\n");
		if (errno == ECHILD) ee_printf("errno=No such child %d\n",res->port.pid);
		if (errno == EINTR) ee_printf("errno=Interrupted\n");
		return 0;
	}
	return 1;
}
#else /* no standard multicore implementation */
#error "Please implement multicore functionality in core_portme.c to use multiple contexts."
#endif /* multithread implementations */
#endif
