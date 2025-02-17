//====================================================================================================================================================
/// Copyright 2025 Lake Orion Robotics FIRST Team 302
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
/// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
/// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

#pragma once

// FRC includes
#include <frc/Timer.h>

// Team 302 includes
#include "utils/logging/debug/LoggableItem.h"
#include "utils/logging/debug/LoggableItemMgr.h"

// Third Party Includes

#include <stdio.h>
#include <string.h>

#ifdef INCLUDE_DATA_TRACE
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

class DataTraceSocket
{
public:
    DataTraceSocket();
    ~DataTraceSocket();

    void Connect(void);
    void Disconnect(void);

    void SendData(void);

    char sendBuffer[1024];

private:
    int status;
    int client_fd;
#ifdef INCLUDE_DATA_TRACE
    struct sockaddr_in serv_addr;
#endif

    bool isConnected = false;

    void extractIpAddress(const char *sourceString, short *ipAddress);

protected:
    frc::Timer m_timer;
};