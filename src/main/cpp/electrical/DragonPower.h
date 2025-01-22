
//====================================================================================================================================================
// Copyright 2025 Lake Orion Robotics FIRST Team 302
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

#pragma once
#include "frc/Timer.h"
#include <frc/PowerDistribution.h>

class DragonPower
{
public:
    static DragonPower *GetInstance();
    void Initialize(int calcFrequency, int logFrequency);
    void CalculateAndLogPower();
    void PrintVoltages();

private:
    DragonPower();
    ~DragonPower() = default;

    static DragonPower *m_dragonPowerInstance;
    int m_calcFrequency = 1000;
    int m_logFrequency = 5000;
    double m_matchTime = 0.0;
    double m_matchWatts = 0.0;
    double m_matchJoules = 0.0;
    double m_matchMaxCurrent = 0.0;
    double m_matchMaxVoltage = 0.0;
    double m_matchMinCurrent = 0.0;
    double m_matchMinVoltage = 0.0;
    frc::PowerDistribution *m_pdp;
    frc::Timer *m_calcTimer;
    frc::Timer *m_logTimer;
};
