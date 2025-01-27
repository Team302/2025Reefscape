
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

#include <feedback/LEDStates.h>
#include <span>

void LEDStates::BlinkingPattern(DragonLeds::Colors c)
{
    if (m_LEDstring->m_ledBuffer.size() > 0)
    {
        if (m_timer > m_blinkPatternPeriod)
            m_timer = 0;

        int blinkState = (m_timer / m_blinkPatternPeriod) % 2;

        if (blinkState == 0)
            m_LEDstring->setBufferAllLEDsColor(m_LEDstring->getColorValues(c));
        else
            m_LEDstring->setBufferAllLEDsBlack();

        m_LEDstring->commitLedData();

        m_timer++;
    }
}

void LEDStates::SolidColorPattern(DragonLeds::Colors c)
{
    m_LEDstring->setBufferAllLEDsColor(m_LEDstring->getColorValues(c));
    m_LEDstring->commitLedData();
}

void LEDStates::AlternatingColorBlinkingPattern(DragonLeds::Colors c)
{
    AlternatingColorBlinkingPattern(c, m_LEDstring->BLACK);
}

void LEDStates::AlternatingColorBlinkingPattern(DragonLeds::Colors c1, DragonLeds::Colors c2)
{
    if (m_LEDstring->m_ledBuffer.size() > 0)
    {
        if (m_timer > 2 * m_altColorBlinkPatternPeriod)
            m_timer = 0;

        int blinkState = (m_timer / m_blinkPatternPeriod) % 2;

        if (blinkState == 0)
            m_LEDstring->setBufferAllLEDsAlternatingColor(m_LEDstring->getColorValues(c1), m_LEDstring->getColorValues(c2));
        else
            m_LEDstring->setBufferAllLEDsAlternatingColor(m_LEDstring->getColorValues(c2), m_LEDstring->getColorValues(c1));

        m_LEDstring->commitLedData();

        m_timer++;
    }
}

void LEDStates::ClosingInChaserPattern(DragonLeds::Colors c)
{
    if (m_LEDstring->m_ledBuffer.size() > 0)
    {
        if (m_timer == 7)
        {
            int halfLength = (m_LEDstring->m_ledBuffer.size() - 1) / 2;
            m_loopThroughIndividualLEDs += m_loopThroughIndividualLEDs < halfLength ? 1 : -m_loopThroughIndividualLEDs;
            int loopout = (m_LEDstring->m_ledBuffer.size() - 1) - m_loopThroughIndividualLEDs;
            auto color = m_colorLoop >= 0 ? m_LEDstring->getColorValues(c) : m_LEDstring->getColorValues(m_LEDstring->BLACK);
            m_colorLoop += m_colorLoop < halfLength ? 1 : -((m_colorLoop * 2) + 1);
            m_LEDstring->m_ledBuffer[m_loopThroughIndividualLEDs].SetRGB(color[0], color[1], color[2]);
            m_LEDstring->m_ledBuffer[loopout].SetRGB(color[0], color[1], color[2]);

            m_LEDstring->commitLedData();
            m_timer = 0;
        }
        m_timer++;
    }
}

void LEDStates::ChaserPattern(DragonLeds::Colors c)
{
    if (m_LEDstring->m_ledBuffer.size() > 0)
    {
        m_loopThroughIndividualLEDs += m_loopThroughIndividualLEDs < static_cast<int>(m_LEDstring->m_ledBuffer.size()) - 1 ? 1 : -m_loopThroughIndividualLEDs;
        if (!m_switchColor)
        {
            color = color == m_LEDstring->getColorValues(c) ? m_LEDstring->getColorValues(DragonLeds::BLACK) : m_LEDstring->getColorValues(c);
        }
        // auto color = m_colorLoop >= 0 ? m_LEDstring->getColorValues(c) : m_LEDstring->getColorValues(DragonLeds::BLACK);

        // m_colorLoop += m_colorLoop < m_LEDstring->m_ledBuffer.size() - 1 ? 1 : -((m_colorLoop * 2) + 1);
        m_switchColor = m_loopThroughIndividualLEDs != static_cast<int>(m_LEDstring->m_ledBuffer.size()) - 1;

        m_LEDstring->m_ledBuffer[m_loopThroughIndividualLEDs].SetRGB(color[0], color[1], color[2]);
        m_LEDstring->commitLedData();
    }
}

void LEDStates::RainbowPattern()
{
    m_LEDstring->setBufferAllLEDsRainbow();
    m_LEDstring->commitLedData();
}

void LEDStates::DiagnosticPattern(frc::DriverStation::Alliance alliancecolor, bool coralInSensor, bool coralOutSensor, bool algaeSensor, bool intakesensor, bool questStatus, bool ll1Status, bool ll2Status, bool pigeonfaults)
{
    if (alliancecolor == frc::DriverStation::Alliance::kBlue)
    {
        m_LEDstring->setSpecificLED(0, m_LEDstring->getColorValues(DragonLeds::BLUE));
    }
    else
    {
        m_LEDstring->setSpecificLED(0, m_LEDstring->getColorValues(DragonLeds::RED));
    }
    if (coralInSensor)
    {
        m_LEDstring->setSpecificLED(1, m_LEDstring->getColorValues(DragonLeds::YELLOW));
    }
    else
    {
        m_LEDstring->setSpecificLED(1, m_LEDstring->getColorValues(DragonLeds::BLACK));
    }
    if (coralOutSensor)
    {
        m_LEDstring->setSpecificLED(2, m_LEDstring->getColorValues(DragonLeds::YELLOW));
    }
    else
    {
        m_LEDstring->setSpecificLED(2, m_LEDstring->getColorValues(DragonLeds::BLACK));
    }
    if (algaeSensor)
    {
        m_LEDstring->setSpecificLED(3, m_LEDstring->getColorValues(DragonLeds::YELLOW));
    }
    else
    {
        m_LEDstring->setSpecificLED(3, m_LEDstring->getColorValues(DragonLeds::BLACK));
    }
    if (intakesensor)
    {
        m_LEDstring->setSpecificLED(4, m_LEDstring->getColorValues(DragonLeds::YELLOW));
    }
    else
    {
        m_LEDstring->setSpecificLED(4, m_LEDstring->getColorValues(DragonLeds::BLACK));
    }
    if (questStatus)
    {
        m_LEDstring->setSpecificLED(5, m_LEDstring->getColorValues(DragonLeds::GREEN));
    }
    else
    {
        m_LEDstring->setSpecificLED(5, m_LEDstring->getColorValues(DragonLeds::RED));
    }
    if (ll1Status)
    {
        m_LEDstring->setSpecificLED(6, m_LEDstring->getColorValues(DragonLeds::GREEN));
    }
    else
    {
        m_LEDstring->setSpecificLED(6, m_LEDstring->getColorValues(DragonLeds::RED));
    }
    if (ll2Status)
    {
        m_LEDstring->setSpecificLED(7, m_LEDstring->getColorValues(DragonLeds::GREEN));
    }
    else
    {
        m_LEDstring->setSpecificLED(7, m_LEDstring->getColorValues(DragonLeds::RED));
    }
    if (pigeonfaults)
    {
        m_LEDstring->setSpecificLED(8, m_LEDstring->getColorValues(DragonLeds::GREEN));
    }
    else
    {
        m_LEDstring->setSpecificLED(8, m_LEDstring->getColorValues(DragonLeds::RED));
    }
    m_LEDstring->commitLedData();
}

void LEDStates::DisabledPattern()
{
    ChaserPattern(DragonLeds::AZUL);
}

void LEDStates::ResetVariables()
{
    m_loopThroughIndividualLEDs = -1;
    m_colorLoop = 0;
    m_timer = 0;
    m_switchColor = false;
}

LEDStates *LEDStates::m_instance = nullptr;
LEDStates *LEDStates::GetInstance()
{
    if (LEDStates::m_instance == nullptr)
    {
        LEDStates::m_instance = new LEDStates();
    }
    return LEDStates::m_instance;
}

void LEDStates::setLEDsOff()
{
    m_LEDstring->setOff();
}

void LEDStates::setLEDsOn()
{
    m_LEDstring->setOn();
}