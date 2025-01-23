
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

#include "electrical/DragonPower.h"
#include "utils/logging/Logger.h"
#include "utils/logging/DragonDataLogger.h"
#include <string>

DragonPower *DragonPower::m_dragonPowerInstance = nullptr;

DragonPower *DragonPower::GetInstance()
{
    if (DragonPower::m_dragonPowerInstance == nullptr)
    {
        DragonPower::m_dragonPowerInstance = new DragonPower();
    }
    return DragonPower::m_dragonPowerInstance;
}

DragonPower::DragonPower()
{    
    int pdpCanID = 0;
    m_pdp = new frc::PowerDistribution(pdpCanID, frc::PowerDistribution::ModuleType::kCTRE);
    m_calcTimer = new frc::Timer();
    m_logTimer = new frc::Timer();
}

void DragonPower::Initialize(int calcFrequency, int logFrequency)
{
    m_calcFrequency = calcFrequency;
    m_logFrequency = logFrequency;
    
}

void DragonPower::CalculateAndLogPower()
{

    // need to determine if we met the calcFrequency threshold
    if (m_pdp != nullptr){
        double currentTimeCalc = m_calcTimer->Get().to<double>();
        if (currentTimeCalc > m_calcFrequency)
        {
            m_calcTimer->Reset();
            m_calcTimer->Start();
            CalculatePowerData(currentTimeCalc);
        }

        double currentTimeLog = m_logTimer->Get().to<double>();
        if (currentTimeLog > m_logFrequency){
            m_logTimer->Reset();
            m_logTimer->Start();
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DragonPower", "Logging", "current time");
        }
    }

}

void DragonPower::CalculatePowerData(double currentTimeCalc)
{
    m_currentCurrent = m_pdp->GetTotalCurrent();
    m_currentVoltage = m_pdp->GetVoltage();
    m_currentPower = m_currentCurrent * m_currentVoltage;
    m_currentEnergy = m_currentPower * currentTimeCalc;
    m_matchPower += m_currentPower;
    m_matchEnergy += m_currentEnergy;
    if (m_currentCurrent > m_matchMaxCurrent)
    {
        m_matchMaxCurrent = m_currentCurrent;
    }
    if (m_currentCurrent < m_matchMinCurrent)
    {
        m_matchMinCurrent = m_currentCurrent;
    }
    if (m_currentVoltage > m_matchMaxVoltage)
    {
        m_matchMaxVoltage = m_currentVoltage;
    }
    if (m_currentVoltage < m_matchMinVoltage)
    {
        m_matchMinVoltage = m_currentVoltage;
    }
    PrintPowerData();
    
}

void DragonPower::PrintPowerData()
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DragonPower", "Current", m_currentCurrent);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DragonPower", "Voltage", m_currentVoltage);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DragonPower", "Power", m_currentPower);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DragonPower", "Energy", m_currentEnergy);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DragonPower", "Match Watts", m_matchPower);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DragonPower", "Match Joules", m_matchEnergy);
}

void DragonPower::LogPowerData()
{
    //DragonDataLogger::LogDoubleData(DragonDataLoggerSignals::StringSignals::ELECTRICAL_VOLTAGE, m_currentVoltage->GetHeadingStateName());
}
