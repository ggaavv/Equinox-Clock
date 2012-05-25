/*******************************************************************************

  Summary:
    
  Description:

*******************************************************************************/

/*******************************************************************************
FileName: 	system_services.h
Copyright ?2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/


#ifndef _SYSTEM_SERVICES_H_
#define _SYSTEM_SERVICES_H_

#include <stdint.h> 
#include <stdbool.h> 
#include <stddef.h>

#if defined (__C32__)
    #include <peripheral/timer.h>
    #include <peripheral/system.h>
#elif defined (__C30__)	
	#include <spi.h>
	#include <timer.h>
#endif
#include "system_debug.h" 
#include "system_random.h" 


// *****************************************************************************
// *****************************************************************************
// Section: System Interface Functions
// *****************************************************************************
// *****************************************************************************

/*********************************************************************
 * Function:        bool SYS_Initialize(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Initializes the system
 *
 * Side Effects:    None
 *
 * Overview:        This function initializes the system.
 *
 * Note:            None
 ********************************************************************/
bool            SYS_Initialize(void);


/*********************************************************************
 * Function:        uint32_t SYS_CLK_ClockGet(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Current system running frequency
 *
 * Side Effects:    None
 *
 * Overview:        This function returns the system running clock frequency.
 *
 * Note:            None
 ********************************************************************/
uint32_t    SYS_CLK_ClockGet(void);


/*********************************************************************
 * Function:        uint32_t SYS_CLK_PeripheralClockGet(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Current peripheral bus frequency
 *
 * Side Effects:    None
 *
 * Overview:        This function returns the peripheral bus clock frequency.
 *
 * Note:            None
 ********************************************************************/
uint32_t    SYS_CLK_PeripheralClockGet(void);

/*********************************************************************
 * Function:        uint32_t SYS_CLK_OscClockGet(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Current system running frequency
 *
 * Side Effects:    None
 *
 * Overview:        This function returns the system running clock frequency
 *                  based on the current oscillator settings
 *
 * Note:            None
 ********************************************************************/
uint32_t    SYS_CLK_OscClockGet(void);


// backward compatible definition for the system clock
#define SystemGetInstructionClock()   SYS_CLK_ClockGet()   

/*********************************************************************
 * Function:        void SYS_WDT_Clear(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function clears the system WatchDog Timer.
 *
 * Note:            None
 ********************************************************************/
void        SYS_WDT_Clear(void);

/*********************************************************************
 * Function:        void SYS_Reboot(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function reboots the system.
 *
 * Note:            None
 ********************************************************************/
void        SYS_Reboot(void);

// *****************************************************************************
// *****************************************************************************
// Section: System Tick Functions
// *****************************************************************************
// *****************************************************************************


typedef unsigned long SYS_TICK;

extern volatile SYS_TICK        _SystemTickCount;
extern volatile unsigned int    _SystemTickResolution;

#define SystemTickDiff(NewTick, StartTick)       (NewTick - StartTick)


typedef const void*    SystemTickHandle;                   // handle to access the System Tick functions     

#if defined (__C30__)

typedef enum
{
	INT_CN   = 27,                  // Change Notice
    INT_INT1 = 28,               // External Interrupt 1
    INT_INT3 = 61,               // External Interrupt 3

}INT_SOURCE;


typedef enum
{

    INT_EXTERNAL_1_VECTOR,            // External Interrupt 1
    INT_EXTERNAL_3_VECTOR,            // External Interrupt 3
    INT_CHANGE_NOTICE_VECTOR,         // Change Notice
    

}INT_VECTOR;

typedef enum
{
    INT_PRIORITY_DISABLED, 
    INT_PRIORITY_LEVEL_1 , 
    INT_PRIORITY_LEVEL_2 , 
    INT_PRIORITY_LEVEL_3 , 
    INT_PRIORITY_LEVEL_4 , 
    INT_PRIORITY_LEVEL_5 , 
    INT_PRIORITY_LEVEL_6 , 
    INT_PRIORITY_LEVEL_7 , 
}INT_PRIORITY;

typedef enum
{
	INT_DISABLED = 0,
    INT_ENABLED  = 1,
}INT_EN_DIS;

#endif

/*********************************************************************
 * Function:        bool SYS_TICK_Initialize(uint32_t sysClock, uint32_t ticksPerSec)
 *
 * PreCondition:    None
 *
 * Input:           sysClock    - system clock frequency, Hz
 *                  ticksPerSec - number of ticks to generate per second
 *
 * Output:          true if initialization succeeded,
 *                  false otherwise
 *
 * Side Effects:    None
 *
 * Overview:        Initializes the system tick counter.
 *
 * Note:            - Normal value for ticksPerSec should be 100 (i.e. interrupt every 10 ms).
 *                  - This implementation uses the core timer.
 *                    It takes exclusive control of the ISR for this resource.
 *                    This may eventually change as it will be integrated at the system level.
 ********************************************************************/
bool SYS_TICK_Initialize(uint32_t sysClock, uint32_t ticksPerSec);

/*********************************************************************
 * Function:        SYS_TICK SYS_TICK_Get(void)
 *
 * PreCondition:    SYS_TICK_Initialize should have been called
 *
 * Input:           None
 *
 * Output:          Current value of the system tick is returned
 *
 * Side Effects:    None
 *
 * Overview:        None
 *
 * Note:            None
 ********************************************************************/
SYS_TICK    SYS_TICK_Get(void);

extern __inline__ SYS_TICK __attribute__((always_inline)) SYS_TICK_Get(void)
{
    return _SystemTickCount;
}


/*********************************************************************
 * Function:        uint32_t SYS_TICK_ResolutionGet(void)
 *
 * PreCondition:    SYS_TICK_Initialize should have been called
 *
 * Input:           None
 *
 * Output:          Current value of the system tick resolution
 *
 * Side Effects:    None
 *
 * Overview:        The function returns the number of system ticks per second
 *
 * Note:            None
 ********************************************************************/
uint32_t        SYS_TICK_ResolutionGet(void);

extern __inline__ uint32_t __attribute__((always_inline)) SYS_TICK_ResolutionGet(void)
{
    return _SystemTickResolution;
}

/*********************************************************************
 * Function:        uint32_t SYS_TICK_TicksPerSecondGet(void)
 *
 * PreCondition:    SYS_TICK_Initialize should have been called
 *
 * Input:           None
 *
 * Output:          Current value of the number of system ticks per second
 *
 * Side Effects:    None
 *
 * Overview:        The function returns the number of system ticks per second
 *
 * Note:            Another name for SYS_TICK_ResolutionGet
 ********************************************************************/
uint32_t        SYS_TICK_TicksPerSecondGet(void);

extern __inline__ uint32_t __attribute__((always_inline)) SYS_TICK_TicksPerSecondGet(void)
{
    return _SystemTickResolution;
}


/*********************************************************************
 * Function:        uint32_t SYS_TICK_HwResolutionGet(void)
 *
 * PreCondition:    SYS_TICK_Initialize should have been called
 *
 * Input:           None
 *
 * Output:          Current value of the hardware timer resolution
 *
 * Side Effects:    None
 *
 * Overview:        The function returns the resolution of the
 *                  system hardware timer
 *
 * Note:            None
 ********************************************************************/
uint32_t        SYS_TICK_HwResolutionGet(void);

extern __inline__ uint32_t __attribute__((always_inline)) SYS_TICK_HwResolutionGet(void)
{
    return SYS_CLK_ClockGet()/2;
}

/*********************************************************************
 * Function:        SYS_TICK SYS_TICK_HwCountGet(void)
 *
 * PreCondition:    SYS_TICK_Initialize should have been called
 *
 * Input:           None
 *
 * Output:          Current value of the hardware system timer used
 *                  to implement the system tick
 *
 * Side Effects:    None
 *
 * Overview:        None
 *
 * Note:            The hardware system timer might overflow pretty fast.
 *                  use SYS_TICK_HwResolutionGet() to see the resolution of the hardware timer
 ********************************************************************/
SYS_TICK    SYS_TICK_HwCountGet(void);

extern __inline__ SYS_TICK __attribute__((always_inline)) SYS_TICK_HwCountGet(void)
{
#if defined (__C32__)
    return ReadCoreTimer();
#elif defined (__C30__)
	return ReadTimer23();
#endif
}


/*********************************************************************
 * Function:        SystemTickHandle SYS_TICK_TimerCreate(void (*tickTimerHandler)(SYS_TICK currSysTick))
 *
 * PreCondition:    SYS_TICK_Initialize should have been called
 *                  tickTimerHandler - valid handler address
 *
 * Input:           tickTimerHandler - handler to be called from the tick ISR
 *
 * Output:          a valid handle if the registration succeeded,
 *                  NULL otherwise
 *
 * Side Effects:    None
 *
 * Overview:        This function creates a timer based on the System Tick.
 *                  It registrates a callback handler with the SystemTick ISR
 *                  The handler will be called from within this ISR at specified rate. 
 *
 * Note:            - All the registered handlers expiring in a current tick
 *                    will be called in turn, sequentially.
 *                    Therefore they have to be as short as possible.
 *                  - The timer handler is called from within an ISR. All the 
 *                    ISR code restrictions apply.
 *                  - The SYS_TICK_TimerCreate() should not be called from within an ISR.
 *                  - The rate of which the System Tick based timer expires is set by the
 *                    SYS_TICK_TimerSetRate();
 *                    
 ********************************************************************/
SystemTickHandle        SYS_TICK_TimerCreate(void (*tickTimerHandler)(SYS_TICK currSysTick));

/*********************************************************************
 * Function:        bool SYS_TICK_TimerDelete(SystemTickHandle tmrHandle)
 *
 * PreCondition:    SYS_TICK_Initialize, SYS_TICK_TimerCreate should have been called
 *
 * Input:           tmrHandle - handle to a Tick Timer to be unregistered from the tick ISR
 *                              The handle should have been obtained by SYS_TICK_TimerCreate()
 *
 * Output:          true if the deletion succeeded, false otherwise
 *
 * Side Effects:    None
 *
 * Overview:        This function deletes a System Tick timer previously created by
 *                  SYS_TICK_TimerCreate() and registered with the System Tick ISR.
 *
 * Note:            The SYS_TICK_TimerDelete() should not be called from within an ISR.
 ********************************************************************/
bool        SYS_TICK_TimerDelete(SystemTickHandle tmrHandle);


/*********************************************************************
 * Function:        bool SYS_TICK_TimerSetRate(SystemTickHandle tmrHandle, unsigned int rate)
 *
 * PreCondition:    SYS_TICK_Initialize, SYS_TICK_TimerCreate should have been called
 *
 * Input:           tmrHandle - handle to a Tick Timer to update the rate for.
 *                              The handle should have been obtained by SYS_TICK_TimerCreate()
 *                  rate      - current timer rate, in System Tick counts
 *
 * Output:          true if the update succeeded, false otherwise
 *
 * Side Effects:    None
 *
 * Overview:        This function sets the rate of a System Tick timer previously created by
 *                  SYS_TICK_TimerCreate() and registered with the System Tick ISR.
 *                 
 *
 * Note:            A timer with rate == 0 is disabled
 ********************************************************************/
bool        SYS_TICK_TimerSetRate(SystemTickHandle tmrHandle, unsigned int rate);

/*********************************************************************
 * Function:        void  SYS_TICK_MsDelay(uint32_t mSec)
 *
 * PreCondition:    SYS_TICK_Initialize should have been called
 *
 * Input:           mSec - delay to achieve
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function delays execution for the specified number of milli seconds
 *
 * Note:            A timer with rate == 0 is disabled
 ********************************************************************/
void        SYS_TICK_MsDelay(uint32_t mSec);

/*********************************************************************
 * Function:        void  SYS_TICK_UsDelay(unsigned long uSec)
 *
 * PreCondition:    SYS_TICK_Initialize should have been called
 *
 * Input:           uSec - delay to achieve, micro seconds
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function delays execution for the specified number of micro seconds
 *
 * Note:            A timer with rate == 0 is disabled
 ********************************************************************/
void        SYS_TICK_UsDelay(uint32_t uSec);

/*********************************************************************
 * Function:        void  SYS_TICK_NsDelay(uint32_t nSec)
 *
 * PreCondition:    SYS_TICK_Initialize should have been called
 *
 * Input:           nSec - delay to achieve, nano seconds
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function delays execution for the specified number of nano seconds
 *
 * Note:            Depending on the system running frequency there is a minimum
 *                  delay that can be achieved.
 *                  See SYS_TICK_HwResolutionGet().
 *                  For example, running at 80MHz, the smallest delay
 *                  that can be achieved is 25 ns.
 *                  These minimum delays are not very accurate.
 ********************************************************************/
void        SYS_TICK_NsDelay(uint32_t nSec);

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupts Interface
// *****************************************************************************
// *****************************************************************************


// system peripheral devices
// Note: listed only the supported ones!!!
typedef enum
{
    SYS_MODULE_ETH_1,
    SYS_MODULE_UART_1,
    SYS_MODULE_UART_2,
    SYS_MODULE_CHAR_LCD,

    // add other
   
} SYS_MODULE_ID;

// system interrupt sources
// Note: listed only the supported ones!!!
typedef enum
{
    /* External interrupt 1 */
    PLIB_INT_SOURCE_EXTERNAL_1,
    
    /* External interrupt 3*/
    PLIB_INT_SOURCE_EXTERNAL_3,
    
    /* Input Change interrupt*/
    PLIB_INT_SOURCE_CHANGE_NOTICE,
    
    /* Ethernet interrupt*/
    PLIB_INT_SOURCE_ETH_1,


    // 
    PLIB_INT_SOURCES    
} PLIB_INT_SOURCE;

typedef PLIB_INT_SOURCE SYS_INT_SOURCE;

typedef enum
{
    PLIB_INT_PRIORITY_LEVEL7 = 7,

    PLIB_INT_PRIORITY_LEVEL6 = 6,

    PLIB_INT_PRIORITY_LEVEL5 = 5,

    PLIB_INT_PRIORITY_LEVEL4 = 4,

    PLIB_INT_PRIORITY_LEVEL3 = 3,

    PLIB_INT_PRIORITY_LEVEL2 = 2,

    PLIB_INT_PRIORITY_LEVEL1 = 1,

    PLIB_INT_PRIORITY_LEVEL0 = 0

} PLIB_INT_PRIORITY_LEVEL;
typedef enum
{
    PLIB_INT_SUBPRIORITY_LEVEL3 = 3,

    PLIB_INT_SUBPRIORITY_LEVEL2 = 2,

    PLIB_INT_SUBPRIORITY_LEVEL1 = 1,

    PLIB_INT_SUBPRIORITY_LEVEL0 = 0

} PLIB_INT_SUBPRIORITY_LEVEL;


typedef PLIB_INT_PRIORITY_LEVEL SYS_INT_PRIORITY;
typedef PLIB_INT_SUBPRIORITY_LEVEL SYS_INT_SUBPRIORITY;



/*********************************************************************
 * Function:        SYS_INT_SOURCE  SYS_INT_Source(SYS_MODULE_ID modId)
 *
 * PreCondition:    None
 *
 * Input:           modId - system module id
 *
 * Output:          the interrupt source corresponding to the requested module. 
 *
 * Side Effects:    None
 *
 * Overview:        This function returns the interrupt source corresponding to a system module.
 *
 * Note:            None
 ********************************************************************/ 
SYS_INT_SOURCE  SYS_INT_Source(SYS_MODULE_ID modId);

extern __inline__ SYS_INT_SOURCE __attribute__((always_inline)) SYS_INT_Source(SYS_MODULE_ID modId)
{
    if(modId == SYS_MODULE_ETH_1)
    {
        return PLIB_INT_SOURCE_ETH_1;
    }

    return -1;
}


/*********************************************************************
 * Function:        bool SYS_INT_SourceDisable(SYS_INT_SOURCE src)
 *
 * PreCondition:    Interrupt system should have been initialized
 *
 * Input:           src - interrupt source to be disabled
 *
 * Output:          true if the corresponding source had been previously enabled.
 *                  false if the corresponding source had been previously disabled. 
 *
 * Side Effects:    None
 *
 * Overview:        This function disables an interrupt source.
 *
 * Note:            - No provision is done in this implementation for multi-threaded operation.
 *                    The assumption is that a specific interrupt source is under a single thread control.
 *                    That is, when SYS_INT_SourceDisable() is called that source is disabled and
 *                    when SYS_INT_SourceRestore() is called the interrupt source is enabled.
 *                  - Some mutex services needed when access from multiple threads needed.
 *                    Or safe counting mechanism to ensure that the interrupts are not inadvertently re-enabled.
 ********************************************************************/ 
bool     SYS_INT_SourceDisable(SYS_INT_SOURCE src);


/*********************************************************************
 * Function:        void SYS_INT_SourceEnable(SYS_INT_SOURCE src)
 *
 * PreCondition:    Interrupt system should have been initialized
 *
 * Input:           src - interrupt source to be enabled
 *
 * Output:          None. 
 *
 * Side Effects:    None
 *
 * Overview:        This function enables an interrupt source.
 *
 * Note:            - No provision is done in this implementation for multi-threaded operation.
 *                    The assumption is that a specific interrupt source is under a single thread control.
 *                    That is, when SYS_INT_SourceDisable() is called that source is disabled and
 *                    when SYS_INT_SourceEnable() is called the interrupt source is enabled.
 *                  - Some mutex services needed when access from multiple threads needed.
 *                    Or safe counting mechanism to ensure that the interrupts are not inadvertently re-enabled.
 ********************************************************************/ 
void     SYS_INT_SourceEnable(SYS_INT_SOURCE src);


/*********************************************************************
 * Function:        bool SYS_INT_SourceRestore(SYS_INT_SOURCE src, bool enabled)
 *
 * PreCondition:    Interrupt system should have been initialized
 *
 * Input:           src     - interrupt source to be restored
 *                  enabled - previous enable status
 *
 * Output:          true if the corresponding source had been enabled.
 *                  false if the corresponding source is disabled. 
 *
 * Side Effects:    None
 *
 * Overview:        This function enables an interrupt source.
 *
 * Note:            - No provision is done in this implementation for multi-threaded operation.
 *                    The assumption is that a specific interrupt source is under a single thread control.
 *                    That is, when SYS_INT_SourceRestore() is called that source is disabled and
 *                    when SYS_INT_SourceRestore() is called the interrupt source is enabled.
 *                  - Some mutex services needed when access from multiple threads needed.
 *                    Or safe counting mechanism to ensure that the interrupts are not inadvertently re-enabled.
 ********************************************************************/ 
bool     SYS_INT_SourceRestore(SYS_INT_SOURCE src, bool enabled);



/*********************************************************************
 * Function:        void SYS_INT_SourceStatusClear(SYS_INT_SOURCE src)
 *
 * PreCondition:    Interrupt system should have been initialized
 *
 * Input:           src - interrupt source to be cleared
 *
 * Output:          None. 
 *
 * Side Effects:    None
 *
 * Overview:        This function clears a pending interrupt source event.
 *                  Performs the hardware acknowledge of the interrupt controller. 
 *
 * Note:            - No provision is done in this implementation for multi-threaded operation.
 *                    The assumption is that a specific interrupt source is under a single thread control.
 ********************************************************************/ 
void     SYS_INT_SourceStatusClear(SYS_INT_SOURCE src);


/*********************************************************************
 * Function:        void SYS_INT_PrioritySet(SYS_INT_SOURCE source, SYS_INT_PRIORITY priority)
 *
 * PreCondition:    Interrupt system should have been initialized
 *
 * Input:           source     - interrupt source to set the priority
 *                  priority   - desired interrupt priority
 *
 * Output:          None. 
 *
 * Side Effects:    None
 *
 * Overview:        This function sets the priority of an interrupt source in the interrupt controller. 
 *
 * Note:            - No provision is done in this implementation for multi-threaded operation.
 *                    The assumption is that a specific interrupt source is under a single thread control.
 ********************************************************************/ 
void     SYS_INT_PrioritySet(SYS_INT_SOURCE source, SYS_INT_PRIORITY priority);


/*********************************************************************
 * Function:        void SYS_INT_SubprioritySet(SYS_INT_SOURCE source, SYS_INT_SUBPRIORITY intSubPri)
 *
 * PreCondition:    Interrupt system should have been initialized
 *
 * Input:           source     - interrupt source to set the sub-priority
 *                  intSubPri  - desired interrupt sub-priority
 *
 * Output:          None. 
 *
 * Side Effects:    None
 *
 * Overview:        This function sets the sub-priority of an interrupt source in the interrupt controller. 
 *
 * Note:            - No provision is done in this implementation for multi-threaded operation.
 *                    The assumption is that a specific interrupt source is under a single thread control.
 ********************************************************************/ 
void     SYS_INT_SubprioritySet(SYS_INT_SOURCE source, SYS_INT_SUBPRIORITY intSubPri);


/*********************************************************************
 * Function:        void SYS_INT_DynamicRegister(  SYS_INT_SOURCE source, void(*handler)(void* param), void* param )
 *
 * PreCondition:    Interrupt system should have been initialized
 *
 * Input:           source   - interrupt source to register
 *                  handler  - desired interrupt handler
 *                  param    - parameter to be poassed to the handler when the interrupt occurs
 *
 * Output:          None. 
 *
 * Side Effects:    None
 *
 * Overview:        This function registers an interrupt handler for a specific interrupt source.
 *                  The registered handler will be executed in the ISR context
 *                  when the hardware interrupt occurs.
 *
 * Note:            No provision is done in this implementation for multi-threaded operation.
 *                  The assumption is that a specific interrupt source is under a single thread control.
 *                  
 *                  Use a NULL handler to perform the de-registration.
 ********************************************************************/ 
void     SYS_INT_DynamicRegister(  SYS_INT_SOURCE source, void(*handler)(void* param), void* param );



// *****************************************************************************
// *****************************************************************************
// Section: System memory allocation Interface
// *****************************************************************************
// *****************************************************************************

/*********************************************************************
 * Function:        void* SystemMalloc(size_t  nBytes)
 *
 * PreCondition:    None
 *
 * Input:           nBytes   - number of bytes to allocate
 *
 * Output:          pointer to the allocated area or NULL if allocation failed. 
 *
 * Side Effects:    None
 *
 * Overview:        This function allocates a memory area of the specified size.
 *                  The returned poiter is properly aligned to contain any standard type.
 *
 * Note:            None
 
 ********************************************************************/ 
void*     SystemMalloc(size_t  nBytes);

#define SystemMalloc    malloc


/*********************************************************************
 * Function:        void* SystemCalloc(size_t nElems, size_t elemSize)
 *
 * PreCondition:    None
 *
 * Input:           nElems   - number of elements to allocate
 *                  elemSize - each element size
 *
 * Output:          pointer to the allocated area or NULL if allocation failed. 
 *
 * Side Effects:    None
 *
 * Overview:        This function allocates a memory area large enough to contain
 *                  nElems, each of the elemSize size.
 *                  The allocated memory area is initialized to 0.
 *                  The returned poiter is properly aligned to contain any standard type.
 *
 * Note:            None
 
 ********************************************************************/ 
void*     SystemCalloc(size_t nElems, size_t elemSize);

#define SystemCalloc    calloc


/*********************************************************************
 * Function:        void SystemFree(void* ptr)
 *
 * PreCondition:    ptr - valid pointer
 *
 * Input:           ptr  - pointer to a previously allocated memory area using SystemMalloc
 *
 * Output:          None 
 *
 * Side Effects:    None
 *
 * Overview:        This function releases a memory area previously allocated.
 *
 * Note:            None
 
 ********************************************************************/ 
void     SystemFree(void* ptr);
#define SystemFree  free


#endif  // _SYSTEM_SERVICES_H_

