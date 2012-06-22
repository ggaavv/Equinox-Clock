/*
 * rtc.c
 *
 *  Created on: 10 Jun 2012
 *      Author: gavin
 */

#include "debug_frmwrk.h"
#include "lpc17xx_rtc.h"


void RTC_IRQHandler(void)
{
	uint32_t secval;

	/* This is increment counter interrupt*/
	if (RTC_GetIntPending(LPC_RTC, RTC_INT_COUNTER_INCREASE))
	{
		secval = RTC_GetTime (LPC_RTC, RTC_TIMETYPE_SECOND);

		/* Send debug information */
		_DBG ("Second: "); _DBD(secval);
		_DBG_("");

		// Clear pending interrupt
		RTC_ClearIntPending(LPC_RTC, RTC_INT_COUNTER_INCREASE);

	}

	/* Continue to check the Alarm match*/
	if (RTC_GetIntPending(LPC_RTC, RTC_INT_ALARM))
	{
		/* Send debug information */
		_DBG_ ("ALARM 10s matched!");

		// Clear pending interrupt
		RTC_ClearIntPending(LPC_RTC, RTC_INT_ALARM);
	}
}


void RTC_time_Init(){

	// Init RTC module
	RTC_Init(LPC_RTC);
	/* Disable RTC interrupt */
    NVIC_DisableIRQ(RTC_IRQn);
    NVIC_SetPriority(RTC_IRQn, 8); // set according to main.c
	RTC_Cmd(LPC_RTC, ENABLE);
	RTC_CalibCounterCmd(LPC_RTC, DISABLE);

    // Set bit 32 of General purpose register 4 to one after configuring time
    // If no 1 set to default time
//	RTC_WriteGPREG(LPC_RTC, 4, 0xaa);
//    uart_send_32_Hex(RTC_ReadGPREG(LPC_RTC, 4));
    if (!(RTC_ReadGPREG(LPC_RTC, 4)==(0xaa)))
    {
    	_DBG("Setting clock time");_DBG("LN:");_DBD(__LINE__);_DBG(" File:");_DBG_(__FILE__);
		/* Enable rtc (starts increase the tick counter and second counter register) */
		RTC_ResetClockTickCounter(LPC_RTC);
		//				 yyyy  mm  dd  Dom Dow  ss  mm  hh
		RTC_time_SetTime(2012,  6, 11, 1,  163, 10, 50, 20);
	        // following line sets the RTC to the date & time this sketch was compiled
//	        RTC.adjust(DateTime(__DATE__, __TIME__)); //TODO: get this to work
    }
//	RTC_CntIncrIntConfig (LPC_RTC, RTC_TIMETYPE_SECOND, ENABLE);

    /* Enable RTC interrupt */
    NVIC_EnableIRQ(RTC_IRQn);

}

void RTC_time_SetTime(uint32_t year, uint32_t month, uint32_t dayOfM, uint32_t dayOfW, uint32_t dayOfY, uint32_t hour, uint32_t min, uint32_t sec) {

	if (year!=NULL) 	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_YEAR, year);
	if (month!=NULL)	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_MONTH, month);
	if (dayOfM!=NULL)	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_DAYOFMONTH, dayOfM);
	if (dayOfW!=NULL)	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_DAYOFMONTH, dayOfW);
	if (dayOfY!=NULL)	RTC_SetTime (LPC_RTC, RTC_TIMETYPE_DAYOFYEAR, dayOfY);
	if (sec!=NULL)		RTC_SetTime (LPC_RTC, RTC_TIMETYPE_SECOND, sec);
	if (min!=NULL)		RTC_SetTime (LPC_RTC, RTC_TIMETYPE_MINUTE, min);
	if (hour!=NULL)		RTC_SetTime (LPC_RTC, RTC_TIMETYPE_HOUR, hour);

}
