
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

#include <string>

#include "utils/DragonPower.h"
#include "utils/logging/debug/Logger.h"

DragonPower *DragonPower::m_dragonPowerInstance = nullptr;
bool calculateInLogger = true; // set this to false to calculate power data via LoggableItem instead of signel

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
}

void DragonPower::CalculatePowerData()
{
    // TODO get some of this data from RoboRio + methods on motors
    // need to determine if we met the calcFrequency threshold
    if (m_pdp != nullptr)
    {
        double currentTimeCalc = m_calcTimer->Get().to<double>();
        m_currentCurrent = m_pdp->GetTotalCurrent();
        m_currentVoltage = m_pdp->GetVoltage();
        m_currentPower = m_currentCurrent * m_currentVoltage;
        m_currentEnergy = m_currentPower * currentTimeCalc;
        m_matchEnergy += m_currentEnergy;
        m_matchWattHours = m_matchEnergy / 3600.0;
        m_calcTimer->Reset();
        m_calcTimer->Start();
    }
}

void DragonPower::LogInformation()
{
    if (!calculateInLogger)
        CalculatePowerData();

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DragonPower", "Current", m_currentCurrent);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DragonPower", "Voltage", m_currentVoltage);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DragonPower", "Power", m_currentPower);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DragonPower", "Energy", m_currentEnergy);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DragonPower", "Match Watt Hours", m_matchWattHours);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DragonPower", "Match Joules", m_matchEnergy);
}

void DragonPower::DataLog(uint64_t timestamp)
{
    if (calculateInLogger)
        CalculatePowerData();

    LogDoubleData(timestamp, DragonDataLoggerSignals::ELECTRICAL_CURRENT, m_currentCurrent);
    LogDoubleData(timestamp, DragonDataLoggerSignals::ELECTRICAL_VOLTAGE, m_currentVoltage);
    LogDoubleData(timestamp, DragonDataLoggerSignals::ELECTRICAL_POWER, m_currentPower);
    LogDoubleData(timestamp, DragonDataLoggerSignals::ELECTRICAL_ENERGY, m_currentEnergy);
}
