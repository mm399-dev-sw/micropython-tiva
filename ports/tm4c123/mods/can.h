/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2014 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifndef MICROPY_INCLUDED_TM4C_CAN_H
#define MICROPY_INCLUDED_TM4C_CAN_H

#include "py/obj.h"
#include "driverlib/can.h"

#define CAN_0 (0)
#define CAN_1 (1)


/**
 *  CAN register struct for easy access 
 *  for register description, see datasheet
*/
typedef struct {                                        /*!< CAN Structure                                                        */
  volatile uint32_t  CTL;                               /*!< CAN Control                                                           */
  volatile uint32_t  STS;                               /*!< CAN Status                                                            */
  volatile uint32_t  ERR;                               /*!< CAN Error Counter                                                     */
  volatile uint32_t  BIT;                               /*!< CAN Bit Timing                                                        */
  volatile uint32_t  INT;                               /*!< CAN Interrupt                                                         */
  volatile uint32_t  TST;                               /*!< CAN Test                                                              */
  volatile uint32_t  BRPE;                              /*!< CAN Baud Rate Prescaler Extension                                     */
  volatile uint32_t  RESERVED;
  volatile uint32_t  IF1CRQ;                            /*!< CAN IF1 Command Request                                               */
  
  union {
    volatile uint32_t  IF1CMSK_CAN_ALT;                 /*!< CAN IF1 Command Mask                                                  */
    volatile uint32_t  IF1CMSK;                         /*!< CAN IF1 Command Mask                                                  */
  };
  volatile uint32_t  IF1MSK1;                           /*!< CAN IF1 Mask 1                                                        */
  volatile uint32_t  IF1MSK2;                           /*!< CAN IF1 Mask 2                                                        */
  volatile uint32_t  IF1ARB1;                           /*!< CAN IF1 Arbitration 1                                                 */
  volatile uint32_t  IF1ARB2;                           /*!< CAN IF1 Arbitration 2                                                 */
  volatile uint32_t  IF1MCTL;                           /*!< CAN IF1 Message Control                                               */
  volatile uint32_t  IF1DA1;                            /*!< CAN IF1 Data A1                                                       */
  volatile uint32_t  IF1DA2;                            /*!< CAN IF1 Data A2                                                       */
  volatile uint32_t  IF1DB1;                            /*!< CAN IF1 Data B1                                                       */
  volatile uint32_t  IF1DB2;                            /*!< CAN IF1 Data B2                                                       */
  volatile uint32_t  RESERVED1[13];
  volatile uint32_t  IF2CRQ;                            /*!< CAN IF2 Command Request                                               */
  
  union {
    volatile uint32_t  IF2CMSK_CAN_ALT;                 /*!< CAN IF2 Command Mask                                                  */
    volatile uint32_t  IF2CMSK;                         /*!< CAN IF2 Command Mask                                                  */
  };
  volatile uint32_t  IF2MSK1;                           /*!< CAN IF2 Mask 1                                                        */
  volatile uint32_t  IF2MSK2;                           /*!< CAN IF2 Mask 2                                                        */
  volatile uint32_t  IF2ARB1;                           /*!< CAN IF2 Arbitration 1                                                 */
  volatile uint32_t  IF2ARB2;                           /*!< CAN IF2 Arbitration 2                                                 */
  volatile uint32_t  IF2MCTL;                           /*!< CAN IF2 Message Control                                               */
  volatile uint32_t  IF2DA1;                            /*!< CAN IF2 Data A1                                                       */
  volatile uint32_t  IF2DA2;                            /*!< CAN IF2 Data A2                                                       */
  volatile uint32_t  IF2DB1;                            /*!< CAN IF2 Data B1                                                       */
  volatile uint32_t  IF2DB2;                            /*!< CAN IF2 Data B2                                                       */
  volatile uint32_t  RESERVED2[21];
  volatile uint32_t  TXRQ1;                             /*!< CAN Transmission Request 1                                            */
  volatile uint32_t  TXRQ2;                             /*!< CAN Transmission Request 2                                            */
  volatile uint32_t  RESERVED3[6];
  volatile uint32_t  NWDA1;                             /*!< CAN New Data 1                                                        */
  volatile uint32_t  NWDA2;                             /*!< CAN New Data 2                                                        */
  volatile uint32_t  RESERVED4[6];
  volatile uint32_t  MSG1INT;                           /*!< CAN Message 1 Interrupt Pending                                       */
  volatile uint32_t  MSG2INT;                           /*!< CAN Message 2 Interrupt Pending                                       */
  volatile uint32_t  RESERVED5[6];
  volatile uint32_t  MSG1VAL;                           /*!< CAN Message 1 Valid                                                   */
  volatile uint32_t  MSG2VAL;                           /*!< CAN Message 2 Valid                                                   */
} periph_can_t;


typedef struct _machine_hard_can_obj_t {
    mp_obj_base_t base;

    uint32_t can_base;                  // base address of can module
    uint32_t periph;                    // address needed for tivaware sysctl functions
    periph_can_t* regs;                 // register access struct pointer (usage: spi_obj.regs->DR)

    u_int32_t mode;                     // TODO add more modes, right now only supports Normal Mode
    mp_uint_t can_id : 8;               // id of the used CAN -> (can_0, can_1)
    bool is_enabled : 1;                // Can enabled
    bool extframe : 1;                  // if true-> use extended identifiers (29 Bits), else standard (11 Bits)
    
    u_int32_t mask;                     // Mask for id filtering
    
    // Timing parameter
    u_int32_t prescaler;
    u_int32_t sjw;
    u_int32_t bs1;
    u_int32_t bs2;
    u_int32_t baudrate;

    tCANMsgObject can_msg_obj[32];      // Message Objects Array
    u_int32_t can_msg_obj_types[32];    // Stores the types of the message Objects 
    mp_obj_t call_back_fun[32];         // Stores Callback funktions

    mp_obj_t call_back_fun_sts;         // Callbackfuntion for bus_off orq
         
} machine_hard_can_obj_t;

extern const mp_obj_type_t machine_hard_can_type;

void CANGenericIntHandler(uint32_t base);

#endif // MICROPY_INCLUDED_TM4C_CAN_H
