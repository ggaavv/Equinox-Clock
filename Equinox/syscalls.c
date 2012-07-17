/*
 * syscalls.c
 *
 *  Created on: 03.12.2009
 *      Author: Martin Thomas, 3BSD license
 */

#include <reent.h>
#include <errno.h>
#include <stdlib.h> /* abort */
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
//#include "rtc.h"

#include "term_io.h"
#include "LPC17xx.h" /* for _get_PSP() from core_cm3.h*/

#undef errno
extern int errno;

char *__env[1] = { 0 };
char **environ = __env;

int _execve(char *name, char **argv, char **env) {
	(void)name; /* avoid warning */
	(void)argv; /* avoid warning */
	(void)env; /* avoid warning */
	errno = ENOMEM;
	return -1;
}
int _fork(void) {
	errno = EAGAIN;
	return -1;
}
int _link(char *old, char *new) {
	(void)old; /* avoid warning */
	(void)new; /* avoid warning */
	errno = EMLINK;
	return -1;
}
int _times(struct tms *buf) {
	(void)buf; /* avoid warning */
	return -1;
}
int _unlink(char *name) {
	(void)name; /* avoid warning */
	errno = ENOENT;
	return -1;
}
int _wait(int *status) {
	(void)status; /* avoid warning */
	errno = ECHILD;
	return -1;
}
#if 1
int _gettimeofday (struct timeval * tp, void * tzvp){
	(void)tp; /* avoid warning */
	(void)tzvp; /* avoid warning */
//	return sys_millis();
//	return time2.unix;
	return -1;
}
#else
int
_gettimeofday (struct timeval * tp, void * tzvp)
{
  struct timezone *tzp = tzvp;
  if (tp)
    {
    /* Ask the host for the seconds since the Unix epoch.  */
#ifdef ARM_RDI_MONITOR
      tp->tv_sec = do_AngelSWI (AngelSWI_Reason_Time,NULL);
#else
      {
        int value;
        asm ("swi %a1; mov %0, r0" : "=r" (value): "i" (SWI_Time) : "r0");
        tp->tv_sec = value;
      }
#endif
      tp->tv_usec = 0;
    }

  /* Return fixed data for the timezone.  */
  if (tzp)
    {
      tzp->tz_minuteswest = 0;
      tzp->tz_dsttime = 0;
    }

  return 0;
}
#endif


#if 0
/* Return a clock that ticks at 100Hz.  */
clock_t
_clock (void)
{
  clock_t timeval;

#ifdef ARM_RDI_MONITOR
  timeval = do_AngelSWI (AngelSWI_Reason_Clock,NULL);
#else
  asm ("swi %a1; mov %0, r0" : "=r" (timeval): "i" (SWI_Clock) : "r0");
#endif
  return timeval;
}
#endif


#if 0
/* Return a clock that ticks at 100Hz.  */
clock_t
_times (struct tms * tp)
{
  clock_t timeval = _clock();

  if (tp)
    {
      tp->tms_utime  = timeval;	/* user time */
      tp->tms_stime  = 0;	/* system time */
      tp->tms_cutime = 0;	/* user time, children */
      tp->tms_cstime = 0;	/* system time, children */
    }

  return timeval;
}
#endif

#if 0
/* Return a clock that ticks at 100Hz.  */
clock_t _times (struct tms * tp){
	clock_t timeval = _clock();

	if (tp){
		tp->tms_utime  = timeval;	/* user time */
		tp->tms_stime  = 0;	/* system time */
		tp->tms_cutime = 0;	/* user time, children */
		tp->tms_cstime = 0;	/* system time, children */
    }
	return timeval;
}
#endif





int _kill(int pid, int sig)
{
	(void)pid;
	(void)sig; /* avoid warnings */
	errno = EINVAL;
	return -1;
}

void _exit(int status)
{
	xprintf("_exit called with parameter %d\n", status);
	while(1) {;}
}

int _getpid(void)
{
	return 1;
}

extern char _end; /* Defined by the linker */
static char *heap_end;

char* get_heap_end(void)
{
	return (char*) heap_end;
}

char* get_stack_top(void)
{
	return (char*) __get_MSP();
	// return (char*) __get_PSP();
}

caddr_t _sbrk(int incr)
{
	char *prev_heap_end;
	if (heap_end == 0) {
		heap_end = &_end;
	}
	prev_heap_end = heap_end;
#if 1
	if (heap_end + incr > get_stack_top()) {
		xprintf("Heap and stack collision\n");
		abort();
	}
#endif
	heap_end += incr;
	return (caddr_t) prev_heap_end;
}

int _close(int file)
{
//	xprintf("%s{\n",__func__);
	(void)file; /* avoid warning */
	return -1;
}

int _fstat(int file, struct stat *st)
{
//	xprintf("%s{\n",__func__);
	(void)file; /* avoid warning */
	st->st_mode = S_IFCHR;
	return 0;
}

int _isatty(int file)
{
//	xprintf("%s{\n",__func__);
	(void)file; /* avoid warning */
	return 1;
}

int _lseek(int file, int ptr, int dir) {
//	xprintf("%s{\n",__func__);
	(void)file; /* avoid warning */
	(void)ptr;  /* avoid warning */
	(void)dir;  /* avoid warning */
	return 0;
}

int _read(int file, char *ptr, int len)
{
//	(void)file; /* avoid warning */
//	(void)ptr;  /* avoid warning */
//	(void)len;  /* avoid warning */
//	xprintf("len=%d,func=%s{\n",len,__func__);
//	printf("len=%d,ptr=%s,func=%s{\n",len,ptr,__func__);
//	switch (file) {
//		case STDIN_FILENO:
//			UART_Receive(LPC_UART0, ptr, len, BLOCKING);
			len = UART_Receive(LPC_UART0, ptr, 1, BLOCKING);
//			xprintf("%s}\n",__func__);
//			break;
//		default:
//			errno = EBADF;
//			return -1;
//	}
	return len;

#if 0
	int n;
	int num = 0;
	switch (file) {
		case STDIN_FILENO:
			for (n = 0; n < len; n++) {
				while ((USART1->SR & USART_FLAG_RXNE) == (uint16_t)RESET) {}
				char c = (char)(USART1->DR & (uint16_t)0x01FF);
				*ptr++ = c;
				num++;
			}
			break;
		default:
			errno = EBADF;
			return -1;
	}
	return num;
#endif
//	return 0;
}

int _write(int file, char *ptr, int len)
{
	int todo;
	(void)file; /* avoid warning */
	for (todo = 0; todo < len; todo++) {
		xputc(*ptr++);
	}
	return len;
}
