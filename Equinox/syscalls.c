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

#include "term_io.h"
#include "LPC17xx.h" /* for _get_PSP() from core_cm3.h*/

#undef errno
extern int errno;

char *__env[1] = { 0 };
char **environ = __env;

int _execve(char *name, char **argv, char **env) {
	errno = ENOMEM;
	return -1;
}
int _fork(void) {
	errno = EAGAIN;
	return -1;
}
int _link(char *old, char *new) {
	errno = EMLINK;
	return -1;
}
int _times(struct tms *buf) {
	return -1;
}
int _unlink(char *name) {
	errno = ENOENT;
	return -1;
}
int _wait(int *status) {
	errno = ECHILD;
	return -1;
}





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
//_gettimeofday(){

//}


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
	(void)file; /* avoid warning */
	return -1;
}

int _fstat(int file, struct stat *st)
{
	(void)file; /* avoid warning */
	st->st_mode = S_IFCHR;
	return 0;
}

int _isatty(int file)
{
	(void)file; /* avoid warning */
	return 1;
}

int _lseek(int file, int ptr, int dir) {
	(void)file; /* avoid warning */
	(void)ptr;  /* avoid warning */
	(void)dir;  /* avoid warning */
	return 0;
}

int _read(int file, char *ptr, int len)
{
	(void)file; /* avoid warning */
	(void)ptr;  /* avoid warning */
	(void)len;  /* avoid warning */
	return 0;
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

#if 0
caddr_t _sbrk_r(int incr)
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

int _close_r(int file)
{
	(void)file; /* avoid warning */
	return -1;
}

int _fstat_r(int file, struct stat *st)
{
	(void)file; /* avoid warning */
	st->st_mode = S_IFCHR;
	return 0;
}

int _isatty_r(int file)
{
	(void)file; /* avoid warning */
	return 1;
}

int _lseek_r(int file, int ptr, int dir) {
	(void)file; /* avoid warning */
	(void)ptr;  /* avoid warning */
	(void)dir;  /* avoid warning */
	return 0;
}

int _read_r(int file, char *ptr, int len)
{
	(void)file; /* avoid warning */
	(void)ptr;  /* avoid warning */
	(void)len;  /* avoid warning */
	return 0;
}

int _write_r(int file, char *ptr, int len)
{
	int todo;
	(void)file; /* avoid warning */
	for (todo = 0; todo < len; todo++) {
		xputc(*ptr++);
	}
	return len;
}
#endif
