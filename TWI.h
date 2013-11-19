/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief  AVR software emulation of TWI slave header file.
 *
 *      This file contains the function prototypes and enumerator definitions
 *      for various configuration parameters for the AVR TWI slave driver.
 *
 *      The driver is not intended for size and/or speed critical code, since
 *      most functions are just a few lines of code, and the function call
 *      overhead would decrease code performance. The driver is intended for
 *      rapid prototyping and documentation purposes for getting started with
 *      the AVR TWI slave.
 *
 *      For size and/or speed critical code, it is recommended to copy the
 *      function contents directly into your application instead of making
 *      a function call.
 *
 * 
 * $Date: 2011-12-07 13:03:43 $  \n
 *
 * Copyright (c) 2011, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#ifndef __TWI_H
#define __TWI_H

#include <avr/io.h>
#include  <avr/interrupt.h>

/*! \brief Definition of pin used as SDA. */
#define SDA PB1

/*! \brief Definition of pin used as SCL. */
#define SCL PB0

/*! \brief Definition of 7 bit slave address. */
#define SLAVE_ADDRESS 0x5D


#define INITIALIZE_SDA()        ( DDRB &= ~ (1 << SDA) )    //!< Clear port. 
#define READ_SDA()              ( (PINB & (1 << SDA)?1:0) )       //!< read pin value
#define READ_SCL()              ( (PINB & (1 << SCL))?1:0 )

//SDA and SCL bit in PORT variable must be 0!
#define SETSDA()  ( DDRB &= ~(1 << SDA) ) //Tristate
#define CLRSDA()  ( DDRB |= (1 << SDA) )  //Pull it low

#define SETSCL()  ( DDRB &= ~(1 << SCL) ) //Tristate
#define CLRSCL()  ( DDRB |= (1 << SCL) )  //Pull it low

/* External interrupt macros. These are device dependent. */
#define INITIALIZE_TWI_INTERRUPT()    (MCUCR |= (1<<ISC01))  //!< Sets falling edge of SDA generates interrupt.
#define ENABLE_TWI_INTERRUPT()        (GIMSK |= (1<<INT0))   //!< Enables SDA interrupt.
#define DISABLE_TWI_INTERRUPT()       (GIMSK &= ~(1<<INT0))  //!< Disables SDA interrupt.
#define CLEAR_TWI_INTERRUPT_FLAG()    (GIFR = (1<<INTF0))    //!< Clears interrupt flag.

#define SDA_vector  INT0_vect  //!< Interrupt vector for SDA pin

// Dedicated general purpose registers. 
register unsigned char TWSR asm("r2");;
register unsigned char TWDR asm("r3");;
register unsigned char TWEA asm("r4");;

//! \name macros for twi state machine
//! @{
# define TWI_SLA_REQ_W_ACK_RTD              0x60    
# define TWI_SLA_DATA_RCV_ACK_RTD           0x80
# define TWI_SLA_DATA_RCV_NACK_RTD          0x88

# define TWI_SLA_REQ_R_ACK_RTD              0xA8
# define TWI_SLA_DATA_SND_ACK_RCV           0xB8
# define TWI_SLA_DATA_SND_NACK_RCV          0xC0
# define TWI_SLA_LAST_DATA_SND_ACK_RCV      0xC8

# define TWI_SLA_REPEAT_START               0xA0
# define TWI_SLA_STOP                       0x68
# define I2C_IDLE                           0x00
//! @}

unsigned char readI2Cslavebyte(void);
void twi_slave_init( void );
void twi_slave_enable( void );

void twi_slave_disable( void );
void receivedata(void);
void senddata(void);
void TWI_state_machine(void);
void GetStartCondition(void);


#endif //__TWI_H