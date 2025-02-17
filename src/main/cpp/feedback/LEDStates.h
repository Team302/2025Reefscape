
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
#include "feedback/DragonLeds.h"
#include <frc/DriverStation.h>
#include <utils/FMSData.h>
#include <vector>

class LEDStates
{
public:
    void setLEDsOn();
    void setLEDsOff();
    void ResetVariables();
    void ChaserPattern(DragonLeds::Colors c);
    void BlinkingPattern(DragonLeds::Colors c);
    void SolidColorPattern(DragonLeds::Colors c);
    void AlternatingColorBlinkingPattern(DragonLeds::Colors c);
    void AlternatingColorBlinkingPattern(DragonLeds::Colors c1, DragonLeds::Colors c2);
    void ClosingInChaserPattern(DragonLeds::Colors c);
    void RainbowPattern();
    void DisabledPattern();
    void DiagnosticPattern(frc::DriverStation::Alliance alliancecolor, bool coralInSensor, bool coralOutSensor, bool algaeSensor, bool intakesensor, bool questStatus, bool ll1Status, bool ll2Status, bool pigeonfaults);
    void BreathingPattern(DragonLeds::Colors c);
    DragonLeds *m_LEDstring = DragonLeds::GetInstance();
    static LEDStates *GetInstance();

private:
    LEDStates() = default;
    ~LEDStates() = default;

    int m_loopThroughIndividualLEDs = -1;
    int m_colorLoop = 0;
    int m_timer = 0;
    bool m_switchColor = false;
    std::array<int, 3U> color = m_LEDstring->getColorValues(DragonLeds::BLACK);
    static LEDStates *m_instance;

    const int m_blinkPatternPeriod = 10;
    const int m_altColorBlinkPatternPeriod = 10;
    const int m_breathePatternPeriod = 10;

    const int m_diagnosticLED0 = 0;
    const int m_diagnosticLED1 = 1;
    const int m_diagnosticLED2 = 2;
    const int m_diagnosticLED3 = 3;
    const int m_diagnosticLED4 = 4;
    const int m_diagnosticLED5 = 5;
    const int m_diagnosticLED6 = 6;
    const int m_diagnosticLED7 = 7;
    const int m_diagnosticLED8 = 8;
};
