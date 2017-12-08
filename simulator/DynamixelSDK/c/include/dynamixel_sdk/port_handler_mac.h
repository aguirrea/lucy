/*******************************************************************************
* Copyright (c) 2016, ROBOTIS CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of ROBOTIS nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Author: Ryu Woon Jung (Leon) */

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_MAC_PORTHANDLERMAC_C_H_
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_MAC_PORTHANDLERMAC_C_H_


#include "port_handler.h"

int portHandlerMac            (const char *port_name);

uint8_t setupPortMac          (int port_num, const int cflag_baud);
uint8_t setCustomBaudrateMac  (int port_num, int speed);
int     getCFlagBaud            (const int baudrate);

double  getCurrentTimeMac     ();
double  getTimeSinceStartMac  (int port_num);

uint8_t openPortMac           (int port_num);
void    closePortMac          (int port_num);
void    clearPortMac          (int port_num);

void    setPortNameMac        (int port_num, const char *port_name);
char   *getPortNameMac        (int port_num);

uint8_t setBaudRateMac        (int port_num, const int baudrate);
int     getBaudRateMac        (int port_num);

int     getBytesAvailableMac  (int port_num);

int     readPortMac           (int port_num, uint8_t *packet, int length);
int     writePortMac          (int port_num, uint8_t *packet, int length);

void    setPacketTimeoutMac     (int port_num, uint16_t packet_length);
void    setPacketTimeoutMSecMac (int port_num, double msec);
uint8_t isPacketTimeoutMac      (int port_num);

#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_MAC_PORTHANDLERMAC_C_H_ */
