/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief  AVR software emulation of TWI slave source file.
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


#include "TWI.h"
#include <avr/io.h>

/* Global variables */
unsigned char incomingBuffer[8]; //!< Incoming buffer
unsigned char outgoingBuffer[6] = {0xAA,0xAB,0xAC,0xAD,0xAE,0xAF}; //!< Preloaded outgoing buffer for testing
unsigned char walker = 0; //!< Walks through the buffer array.

/*! \brief initialize twi slave mode
 */
void twi_slave_init( void )
{
    INITIALIZE_SDA(); 
    INITIALIZE_TWI_INTERRUPT();
    TWEA = 1;
    
    TWSR = I2C_IDLE;
}


/*! \brief enable twi slave
 */
void twi_slave_enable( void )
{
  CLEAR_TWI_INTERRUPT_FLAG();
  ENABLE_TWI_INTERRUPT();
}


/*! \brief disable twi slave
 */
void twi_slave_disable( void )
{
  DISABLE_TWI_INTERRUPT();
  CLEAR_TWI_INTERRUPT_FLAG();
}


/*! \brief Interrupt service routine for negative egde on SDA
 */
ISR(SDA_vector) {
  volatile unsigned char retval;
  
  if(TWSR == I2C_IDLE)
  {
    GetStartCondition();
  }
  
  //!Call the TWi state machine
  TWI_state_machine();
  CLEAR_TWI_INTERRUPT_FLAG();
  ENABLE_TWI_INTERRUPT();

  
}

/*! \brief Read the slave byte after start condition
 */
inline unsigned char readI2Cslavebyte(void)
{
  unsigned char index = 0;
  unsigned char val = 0;
  unsigned char cPin = 0;
  
  
  //Let SCL go low first. MCU comes here while SCL is still high
  while(READ_SCL());
  
  //!read 8 bits from master, respond with ACK.
  //!SCL could be high or low depending on CPU speed
  for(index = 0;index < 8; index++)
  {
    
    while(!READ_SCL());
    cPin = READ_SDA();
    
    val = (val<<1) | cPin;
    while(READ_SCL())
    {
      //PORTB ^= (1<<(PB4)); //timing debug
      //if SDA changes while SCL is high, it indicates STOP or START
      if((val & 1)!= cPin)
      {
        if(READ_SDA())
          TWSR = TWI_SLA_STOP;
        else
          TWSR = TWI_SLA_REPEAT_START;
        return 0;
        //!return READ_SDA()?I2C_STOP_DETECTED:I2C_START_DETECTED;
      } 
      else
        cPin = READ_SDA();
      //PORTB ^= (1<<(PB4)); //timing debug
    }
  }
  
  //!Send ACK, SCL is low now
  if((val & 0xFE) == (SLAVE_ADDRESS << 1))
  {
    
    CLRSDA();
    while(!READ_SCL());
    while(READ_SCL());
    SETSDA();
    CLRSCL(); //!!!clock_stretching start
  }
  else
  {
    TWSR = I2C_IDLE;
    return 0;
  }
  return val; 
}

/*! \brief TWI slave send data
 */
inline void senddata(void)
{
  unsigned char index;
  for(index = 0;index < 8; index++)
  {
    while(READ_SCL());
    //PORTB ^= (1<<(PB4));
    if((TWDR >> (7 - index))&1)
      SETSDA();
    else
      CLRSDA();
    //PORTB ^= (1<<(PB4)); //timing debug
    SETSCL(); //!!! clock_stretching stop
    while(!READ_SCL());
  }
  //!See if we get ACK or NACk
  while(READ_SCL());
  
  //!tristate the pin to see if ack comes or not
  SETSDA();
  
  while(!READ_SCL());
  if(!READ_SDA())
    TWSR = TWI_SLA_DATA_SND_ACK_RCV;
  else
    TWSR = TWI_SLA_DATA_SND_NACK_RCV;
}


/*! \brief TWI slave receive data
 */
inline void receivedata(void)
{
  unsigned char index;
  TWDR = 0;
  SETSCL(); //!!! clock_stretching stop
  
  //!read 8 bits from master, respond with ACK.
  //!SCL could be high or low depending on CPU speed
  for(index = 0;index < 8; index++)
  {
    while(!READ_SCL());
    TWDR = (TWDR<<1) | READ_SDA();
    while(READ_SCL())
    {
      //!if SDA changes while SCL is high, it indicates STOP or START
      if((TWDR & 1)!= READ_SDA())
      {
        if(READ_SDA())
          TWSR = TWI_SLA_STOP;
        else
          TWSR = TWI_SLA_REPEAT_START;
        return;
      } 
    }
  }
  
  if(TWEA)
  { 
    //!Send ACK, SCL is low now
    CLRSDA();
    while(!READ_SCL());
    while(READ_SCL());
    SETSDA();
    TWSR = TWI_SLA_DATA_RCV_ACK_RTD;
    TWEA = 0;
  }
  else
  {
    TWSR = TWI_SLA_DATA_RCV_NACK_RTD;
  }
  
  
  
}


/*! \brief TWI state machine software algorithm that emulates the hardware TWI state machine.
 */
void TWI_state_machine(void)
{
    //! get current state
START: 
    //! \brief lock twi task              
    switch (TWSR) 
    {    
        /*! Own SLA_W has been received; 
         *! ACK has been returned
         */
        case TWI_SLA_REQ_W_ACK_RTD: 
            walker = 0;
            incomingBuffer[walker++] = TWDR; 
            TWEA = 1;
            receivedata();
            goto START;
            
            break;
            
            
        //! data recieved, NACK has been returned
        case TWI_SLA_DATA_RCV_NACK_RTD:
            TWSR = I2C_IDLE;
            TWEA = 1;
            
            break;
        
        //! data recieved, ack has been returned
        case TWI_SLA_DATA_RCV_ACK_RTD:
            incomingBuffer[walker++] = TWDR;
            TWEA = 1;
            
            receivedata();
            goto START;
            
            break;


        /*! Own SLA_R has been received; 
         *! ACK has been returned
         */
        case TWI_SLA_REQ_R_ACK_RTD:
            //PORTB ^= (1<<(PB4)); //timing debug
            walker = 0;  
            TWDR = outgoingBuffer[walker++];
            TWEA = 1;
            
            senddata();
            goto START;
            
            break;

        //! data has been transmitted, ACK has been received.
        case TWI_SLA_DATA_SND_ACK_RCV:
            TWDR = outgoingBuffer[walker++];
            TWEA = 1;
            
            senddata();
            goto START;
            
            break;
            
        //! last data has been transmitted, ACK has been received.
        case TWI_SLA_LAST_DATA_SND_ACK_RCV:
         
        //! data has been transmitted, NACK has been received.
        case TWI_SLA_DATA_SND_NACK_RCV:
            TWEA = 1;
            TWSR = I2C_IDLE;
            
            break;

        //! met stop or repeat start
        case TWI_SLA_STOP:
              //! return to idle state
              TWEA = 1;
              TWSR = I2C_IDLE;
          
            break;
        case TWI_SLA_REPEAT_START:  
          GetStartCondition();
          goto START;
          
        //! Idle or bus error
        case I2C_IDLE:
        default:
            TWEA = 1; 
            break;        
    }
}

/*! \brief Identify start condition
 */
inline void GetStartCondition(void)
{
  unsigned char retval = 0;
  //!Make sure it is the start by checking SCL high when SDA goes low
  if(READ_SCL())
  {
    DISABLE_TWI_INTERRUPT();
  }
  else //!false trigger; exit the ISR
  {
    CLEAR_TWI_INTERRUPT_FLAG();
    ENABLE_TWI_INTERRUPT();
    return;
  }
  //!lop for one or several start conditions before a STOP
  if(TWSR == I2C_IDLE || TWSR == TWI_SLA_REPEAT_START)
  {
    retval = readI2Cslavebyte(); //!read address
    if(retval == 0)//!STOP or otehr address received
    {
      TWSR = I2C_IDLE;
      CLEAR_TWI_INTERRUPT_FLAG();
      ENABLE_TWI_INTERRUPT();
      return;
    }
    else
    {
      if(retval & 1)
        TWSR = TWI_SLA_REQ_R_ACK_RTD;
      else
        TWSR = TWI_SLA_REQ_W_ACK_RTD;
    }
  }
  TWDR = retval;
  
}
