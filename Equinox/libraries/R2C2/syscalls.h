/*
 * syscalls.c
 *
 *  Created on: 03.12.2009
 *      Author: Martin Thomas, 3BSD license
 */
 /* MOdified by Sagar G V, Feb 2011.
 Ported to LPC17xx.
*/

#ifndef __SYSCALLS_H__
#define __SYSCALLS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <reent.h>
#include <errno.h>
#include <stdlib.h> /* abort */
#include <sys/types.h>
#include <sys/stat.h>


#include "LPC17xx.h" /* for _get_PSP() from core_cm3.h*/




#ifdef __cplusplus
}
#endif

#endif