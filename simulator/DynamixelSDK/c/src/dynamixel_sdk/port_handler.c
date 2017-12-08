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

#if defined(__linux__)
#include "port_handler.h"
#include "port_handler_linux.h"

int     portHandler         (const char *port_name) { return portHandlerLinux(port_name); }

uint8_t openPort            (int port_num) { return openPortLinux(port_num); }
void    closePort           (int port_num) { closePortLinux(port_num); }
void    clearPort           (int port_num) { clearPortLinux(port_num); }

void    setPortName         (int port_num, const char *port_name) { setPortNameLinux(port_num, port_name); }
char   *getPortName         (int port_num) { return getPortNameLinux(port_num); }

uint8_t setBaudRate         (int port_num, const int baudrate) { return setBaudRateLinux(port_num, baudrate); }
int     getBaudRate         (int port_num) { return getBaudRateLinux(port_num); }

int     getBytesAvailable   (int port_num) { return getBytesAvailableLinux(port_num); }

int     readPort            (int port_num, uint8_t *packet, int length) { return readPortLinux(port_num, packet, length); }
int     writePort           (int port_num, uint8_t *packet, int length) { return writePortLinux(port_num, packet, length); }

void    setPacketTimeout    (int port_num, uint16_t packet_length) { setPacketTimeoutLinux(port_num, packet_length); }
void    setPacketTimeoutMSec(int port_num, double msec) { setPacketTimeoutMSecLinux(port_num, msec); }
uint8_t isPacketTimeout     (int port_num) { return isPacketTimeoutLinux(port_num); }

#elif defined(__APPLE__)
#include "port_handler.h"
#include "port_handler_mac.h"

int     portHandler         (const char *port_name) { return portHandlerMac(port_name); }

uint8_t openPort            (int port_num) { return openPortMac(port_num); }
void    closePort           (int port_num) { closePortMac(port_num); }
void    clearPort           (int port_num) { clearPortMac(port_num); }

void    setPortName         (int port_num, const char *port_name) { setPortNameMac(port_num, port_name); }
char   *getPortName         (int port_num) { return getPortNameMac(port_num); }

uint8_t setBaudRate         (int port_num, const int baudrate) { return setBaudRateMac(port_num, baudrate); }
int     getBaudRate         (int port_num) { return getBaudRateMac(port_num); }

int     getBytesAvailable   (int port_num) { return getBytesAvailableMac(port_num); }

int     readPort            (int port_num, uint8_t *packet, int length) { return readPortMac(port_num, packet, length); }
int     writePort           (int port_num, uint8_t *packet, int length) { return writePortMac(port_num, packet, length); }

void    setPacketTimeout    (int port_num, uint16_t packet_length) { setPacketTimeoutMac(port_num, packet_length); }
void    setPacketTimeoutMSec(int port_num, double msec) { setPacketTimeoutMSecMac(port_num, msec); }
uint8_t isPacketTimeout     (int port_num) { return isPacketTimeoutMac(port_num); }

#elif defined(_WIN32) || defined(_WIN64)
#define WINDLLEXPORT
#include "port_handler.h"
#include "port_handler_windows.h"

int     portHandler         (const char *port_name) { return portHandlerWindows(port_name); }

uint8_t openPort            (int port_num) { return openPortWindows(port_num); }
void    closePort           (int port_num) { closePortWindows(port_num); }
void    clearPort           (int port_num) { clearPortWindows(port_num); }

void    setPortName         (int port_num, const char *port_name) { setPortNameWindows(port_num, port_name); }
char   *getPortName         (int port_num) { return getPortNameWindows(port_num); }

uint8_t setBaudRate         (int port_num, const int baudrate) { return setBaudRateWindows(port_num, baudrate); }
int     getBaudRate         (int port_num) { return getBaudRateWindows(port_num); }

int     readPort            (int port_num, uint8_t *packet, int length) { return readPortWindows(port_num, packet, length); }
int     writePort           (int port_num, uint8_t *packet, int length) { return writePortWindows(port_num, packet, length); }

void    setPacketTimeout    (int port_num, uint16_t packet_length) { setPacketTimeoutWindows(port_num, packet_length); }
void    setPacketTimeoutMSec(int port_num, double msec) { setPacketTimeoutMSecWindows(port_num, msec); }
uint8_t isPacketTimeout     (int port_num) { return isPacketTimeoutWindows(port_num); }

#endif
