/*************************************************************************
**  Sony Unilink Bus Interface (USART Procedures)
**  by Michael Wolf
**
**  Released under GNU GENERAL PUBLIC LICENSE
**	See LICENSE.TXT for details
**
**  Revision History
**
**  when         what  who	why
**
**  2003-08-05   2.01  MIC  complete code revision
**													* changed USART receive procedure
**													- bugfix in time update command
**  2003-09-06   2.02  MIC  * changed USART receive procedure
**                          + added serial commands to change the playmodes
**  2004-01-05   2.03  MIC  + added serial 'L' command to receive list items
**													- bufix in BANK mode command
**  2005-02-03   2.04  MIC  - removed all deprecaded avr-libc macros to be
**                            combatible with avr-libc v1.2.1 and higher
**************************************************************************/
#include <avr/pgmspace.h>

#define USART_INT

#ifdef USART_INT
/* UART Buffer Defines */

#define USART_TX_BUFFER_SIZE 128     /* 2,4,8,16,32,64,128 or 256 bytes */
#define USART_TX_BUFFER_MASK ( USART_TX_BUFFER_SIZE - 1 )
#if ( USART_TX_BUFFER_SIZE & USART_TX_BUFFER_MASK )
	#error TX buffer size is not a power of 2
#endif

static unsigned char USART_TxBuf[USART_TX_BUFFER_SIZE];
static volatile unsigned char USART_TxHead;
static volatile unsigned char USART_TxTail;

// USART Transmit buffer
#if defined(__AVR_ATmega162__)
SIGNAL(USART0_UDRE_vect)
#else
SIGNAL(UART_UDRE_vect)
#endif
{
	unsigned char tmptail;
	/* Check if all data is transmitted */
	if ( USART_TxHead != USART_TxTail )
	{
		/* Calculate buffer index */
		tmptail = ( USART_TxTail + 1 ) & USART_TX_BUFFER_MASK;
		USART_TxTail = tmptail;      /* Store new index */
		#if defined(__AVR_ATmega162__)
		UDR0 = USART_TxBuf[tmptail];  /* Start transmition */
		#else
		UDR = USART_TxBuf[tmptail];  /* Start transmition */
		#endif
	}
	else
	{
	#if defined(__AVR_ATmega162__)
		UCSR0B &= ~(1<<UDRIE0);         /* Disable UDRE interrupt */
	#elif defined(__AVR_AT90S8515__)
		UCR &= ~(1<<UDRIE);        /* Disable UDRE interrupt */
	#else
		UCR &= ~(1<<UDRIE);         /* Disable UDRE interrupt */
	#endif
	}
};
#endif

// USART receive interrupt
#if defined(__AVR_ATmega162__)
SIGNAL(USART0_RXC_vect)
#else
SIGNAL(UART_RX_vect)
#endif
{
// (0x0a) = LF
// (0x0d) = CR
	unsigned char temp_UDR;
#if defined(__AVR_ATmega162__)
	temp_UDR = UDR0;
#else
	temp_UDR = UDR;
#endif
/*
	if(temp_UDR == '/'){
		no_usart_rx_chars = 0;
		usart_rx_index = 0;
		usart_puts("rx buffer reset. \r");
		return;
	}
*/
#ifdef CANUSB_COMPATIBLE
	if(!flags.can_usart_recieve_complete)
	{
		no_usart_rx_chars++;
		if(temp_UDR==CR)
		{
			if(usart_rx_index == 0)
			{
				usart_putc(CR);
				return;
			}
			usart_rx_buffer[usart_rx_index] = temp_UDR;
			flags.can_usart_recieve_complete = true;
			usart_rx_index = 0;
		}
		else
		{
   			usart_rx_buffer[usart_rx_index] = temp_UDR;		            // put received char in buffer
   			usart_rx_index++;
		}
	}
#else
	if(!flags.usart_recieve_complete&&!flags.can_usart_recieve_complete)
	{
		no_usart_rx_chars++;
		if(temp_UDR==CR)
		{
			usart_rx_buffer[usart_rx_index] = temp_UDR;
			flags.usart_recieve_complete = true;
			usart_rx_index = 0;
		}
		else
		{
   			usart_rx_buffer[usart_rx_index] = temp_UDR;		            // put received char in buffer
   			usart_rx_index++;
		}
		usart_putc(temp_UDR);
	}
#endif
};// end of UART receive interrupt

void usart_putc(unsigned char byte)
{
#ifdef USART_INT
	unsigned char tmphead;
	/* Calculate buffer index */
	tmphead = ( USART_TxHead + 1 ) & USART_TX_BUFFER_MASK; /* Wait for free space in buffer */
	while ( tmphead == USART_TxTail );
	USART_TxBuf[tmphead] = byte;           /* Store data in buffer */
	USART_TxHead = tmphead;                /* Store new index */
	#if defined(__AVR_ATmega162__)
	UCSR0B |= (1<<UDRIE0);                    /* Enable UDRE interrupt */
	#else
	UCR |= (1<<UDRIE);                    /* Enable UDRE interrupt */
	#endif
#else
	#if defined(__AVR_ATmega162__)
	while ((UCSR0A & _BV(UDRE0)) != _BV(UDRE0));			// wait for USART to become available
	UDR0 = byte;							// send character
	#else
	while ((UCR & _BV(UDRE)) != _BV(UDRE));		// wait for USART to become available
	UDR = byte;							// send character
	#endif
#endif
}; //end usart_putc

//void usart_puts (uint8_t *tx_string)
void usart_puts (char *tx_string)
{
    while (*tx_string)
        usart_putc (*tx_string++);	// send string char by char
}

void usart_init(void)					                	// init USART
{
#if defined(__AVR_ATmega162__)
	/* Enable UART receiver and transmitter */
	UCSR0B = ((1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0)|(0<<UDRIE0));
	/* Set frame format: 8 data 1stop */
	UCSR0C = (1<<URSEL0)|(0<<USBS0)|(3<<UCSZ00);
    //UBRR0 = (unsigned char)UART_BAUDRATE;		// set baud rate
    UBRR0H = (UART_BAUDRATE>>8);
	UBRR0L = UART_BAUDRATE;
#elif defined(__AVR_AT90S8515__)
	UBRRH = UART_BAUDRATE>>8;		// set baud rate
	UBRRL = UART_BAUDRATE;
	UCSRB =((1<<RXCIE)|(0<<TXCIE)|(0<<UDRIE)|(1<<RXEN)|(1<<TXEN)|(0<<UCSZ2)|(0<<RXB8)|(0<<TXB8));
	// enable Rx & Tx, enable Rx interrupt
	UCSRC =((1<<URSEL)|(0<<UMSEL)|(0<<UPM1)|(0<<UPM0)|(0<<USBS)|(1<<UCSZ1)|(1<<UCSZ0)|(0<<UCPOL));
#else
	UBRR = UART_BAUDRATE;		// set baud rate
	UCR = 0x98;			            // enable Rx & Tx, enable Rx interrupt
#endif
	no_usart_rx_chars = 0;
#ifdef USART_INT
  	/* Flush buffer */
  	unsigned char x;
	x = 0;
	USART_TxTail = x;
	USART_TxHead = x;
#endif
#ifndef CANUSB_COMPATIBLE
	usart_puts(PSTR("Uart ini.\n"));
#endif
}; // end UART_INIT

void usart_rx_proc(void)
{

	#if defined(__AVR_ATmega162__)
	//UCSR0C = 0x88;		                                        // disable UART Rx
	UCSR0B = ((1<<RXCIE0)|(0<<RXEN0)|(1<<TXEN0));
	#elif defined(__AVR_AT90S8515__)
	UCSR0B = ((1<<RXCIE0)|(0<<RXEN)|(1<<TXEN));
	#else
	UCR = 0x88;	                                        // disable UART Rx
	#endif
//usart_putc('r');

#ifndef CANUSB_COMPATIBLE
	unsigned char loc_temp = 0;
    switch(usart_rx_buffer[0])
    {
    	case 'c': //can message
    		flags.can_usart_recieve_complete = true;
 //   		usart_putc('s');
    		break;
        case 'h':
        	usart_puts(PSTR("line 1."));
        	usart_puts(PSTR("line 2."));
        	break;
        case 't':
            // Disc number
            unilink_timeinfo[7] = ((((usart_rx_buffer[1])+1)&0x0F)<<4) | 0x0E;
            // for track number, make 1 byte BCD from two byte ASCII
            unilink_timeinfo[4] = ((usart_rx_buffer[2] - 0x30) << 4 |
								(usart_rx_buffer[3] - 0x30)); // one byte track number from 2 byte ASCII
						if(flags.interface_mode)
							unilink_timeinfo[4] = bcd2hex(unilink_timeinfo[4]);// in MD mode make HEX from BCD
           // its the same for minutes
            unilink_timeinfo[5] = ((usart_rx_buffer[4]-0x30) << 4 |
							(usart_rx_buffer[5] - 0x30));// one byte minutes from 2 byte ASCII
						if(flags.interface_mode)
							unilink_timeinfo[5] = bcd2hex(unilink_timeinfo[5]);// in MD mode make HEX from BCD
           // seconds
            unilink_timeinfo[6] = ((usart_rx_buffer[6] - 0x30) << 4 |
								(usart_rx_buffer[7]-0x30));	// one byte seconds from 2 byte ASCII
            unilink_timeinfo[1] = unilink_ownaddr;              // set TAD
			update_display.time = true;								// set flag to force a time update
			break;
        case 'D':
			// disc name
			for(loc_temp = 0; (loc_temp < 48) ; loc_temp++ )    	// copy disc name to name buffer
			{
				if(usart_rx_buffer[loc_temp +1] == '~') break;
				unilink_discname[loc_temp] = usart_rx_buffer[loc_temp +1];
			};
			for(; loc_temp < 48; loc_temp++)
				unilink_discname[loc_temp] = ' '; 					// clear track name buffer
			update_display.discname = true;								// set flag to force a discname update
			break;
        case 'T':
            // track name
            for(loc_temp = 0; (loc_temp < 48) ; loc_temp++ )    	// copy track name to name buffer
            {
                if(usart_rx_buffer[loc_temp +1] == '~') break;
                unilink_trackname[loc_temp] = usart_rx_buffer[loc_temp +1];
            };
			for(; loc_temp < 48; loc_temp++) unilink_trackname[loc_temp] = ' '; // clear track name buffer
			update_display.trackname = true;									// set flag to force a track name update
			break;
		case 'r':
			// Repeat mode
			unilink_playmodes.repeat = usart_rx_buffer[1] & 0x03;
			change_mode(0x08, 0x00, 0x00);
			break;
		case 's':
			// Shuffle mode
			unilink_playmodes.shuffle = usart_rx_buffer[1] & 0x03;
			change_mode(0x00, 0x80, 0x00);
			break;
		case 'i':
			// Intro mode
			unilink_playmodes.intro = usart_rx_buffer[1] & 0x01;
			change_mode(0x20, 0x00, 0x00);
			break;
		case 'd':
		    // DScan mode
			unilink_playmodes.dscan = usart_rx_buffer[1] & 0x01;
			change_mode(0x80, 0x00, 0x00);
			break;
		case 'b':
		    // Bank mode
			unilink_playmodes.bank = usart_rx_buffer[1] & 0x03;
			change_mode(0x00, 0x00, 0x20);
			break;
		case 'L':
			// List item
			if ( (usart_rx_buffer[1]&0x0F) > 9)
			usart_rx_buffer[1] = '9';
			for(loc_temp = 0; (loc_temp < 8) ; loc_temp++ )
			{
				if(usart_rx_buffer[loc_temp +2] == '~') break;
				unilink_list[ usart_rx_buffer[1]&0x0F ] [loc_temp] = usart_rx_buffer[loc_temp+2];
			};
			for(; loc_temp < 8; loc_temp++)
				unilink_list[ usart_rx_buffer[1]&0x0F ] [loc_temp] = ' '; // clear list buffer
			break;
		case '(':
			// Prev track
			create_message_and_tx(0,0,0x21,0x7E,0x40);
			create_message_and_tx(1,0,0x21,0x7E,0x4A);
			break;
		case ')':
			// Next track
			create_message_and_tx(0,0,0x21,0x7E,0x50);
			create_message_and_tx(1,0,0x21,0x7E,0x5A);
			break;
		case '<':
			// Seek -
			create_message_and_tx(0,0,0x21,0x7E,0x80);
			create_message_and_tx(1,0,0x21,0x7E,0x8A);
			break;
		case '>':
			// Seek +
			create_message_and_tx(0,0,0x21,0x7E,0x90);
			create_message_and_tx(1,0,0x21,0x7E,0x9A);
			break;
		case '-':
			// Volume -
			create_message_and_tx(0,0,0x21,0x70,0x10);
			create_message_and_tx(1,0,0x21,0x70,0x1A);
			break;
		case '+':
			// Volume +
			create_message_and_tx(0,0,0x21,0x70,0x20);
			create_message_and_tx(1,0,0x21,0x70,0x2A);
			break;
		case 'S':
			// Source
			create_message_and_tx(0,0,0x21,0x10,0x80);
			create_message_and_tx(1,0,0x21,0x10,0x8A);
			break;
		case 'f':
			// Off
			create_message_and_tx(0,0,0x21,0x10,0x50);
			create_message_and_tx(1,0,0x21,0x10,0x5A);
			break;
		case 'm':
			// Mute
			create_message_and_tx(0,0,0x21,0x71,0x00);
			create_message_and_tx(1,0,0x21,0x71,0x0A);
			break;
#ifdef RAW_COMMAND
        case '*':
            // raw command mode
            asm volatile ("nop");
            unsigned char raw[13] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
            for(loc_temp = 0; loc_temp < 13; loc_temp++)
							raw[loc_temp] = usart_rx_buffer[loc_temp +1];
#endif
#if !defined(BUS_LOGGING) && defined(YAMPP3)
				/* When bus logging is disabled and Yampp3 control is enabled, the interface will wait for
				   a signal from Yampp to send a "Play" when the Yampp has finished power-up and when
				   interface status is in play mode */
		case '!':
			if(unilink_status == 0x00) usart_putc(Start_Tag);
			if(unilink_status == 0x00) usart_putc(Play);
			if(unilink_status == 0x00) usart_putc(CR);
			break;
		#endif
    }; // end switch
#else
	flags.can_usart_recieve_complete = true;
#endif
                                       	// enable UART Rx
#if defined(__AVR_ATmega162__)
	//UCSR0C = 0x98;		                                        	// enable UART Rx
	UCSR0B = ((1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0));
#elif defined(__AVR_AT90S8515__)
	UCSR0B = ((1<<RXCIE0)|(1<<RXEN)|(1<<TXEN));
#else
	UCR = 0x98;
#endif
}; // end of UART_RX_PROC

void usart_byte2ascii (uint8_t tx_byte)
{
    // send high nibble
    usart_putc (((tx_byte >> 4) <
	       10) ? ((tx_byte >> 4) & 0x0f) + 48 : ((tx_byte >> 4) & 0x0f) +
	      55);
    // send low nibble
    usart_putc (((tx_byte & 0x0f) <
	       10) ? (tx_byte & 0x0f) + 48 : (tx_byte & 0x0f) + 55);
}

