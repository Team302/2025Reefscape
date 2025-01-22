
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
            double current = m_pdp->GetTotalCurrent();
            double voltage = m_pdp->GetVoltage();
            double power = current * voltage;
            double joules = power * currentTimeCalc;
            m_matchWatts += power;
            m_matchJoules += joules;
            if (current > m_matchMaxCurrent)
            {
                m_matchMaxCurrent = current;
            }
            if (current < m_matchMinCurrent)
            {
                m_matchMinCurrent = current;
            }
            if (voltage > m_matchMaxVoltage)
            {
                m_matchMaxVoltage = voltage;
            }
            if (voltage < m_matchMinVoltage)
            {
                m_matchMinVoltage = voltage;
            }

            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DragonPower", "Current", current);
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DragonPower", "Voltage", voltage);
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DragonPower", "Power", power);
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DragonPower", "Joules", joules);
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DragonPower", "Match Watts", m_matchWatts);
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DragonPower", "Match Joules", m_matchJoules);
        }
        double currentTimeLog = m_logTimer->Get().to<double>();
        if (currentTimeLog > m_logFrequency){
            m_logTimer->Reset();
            m_logTimer->Start();
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DragonPower", "Logging", "current time");
        }
    }

}

void DragonPower::PrintVoltages()
{
    double voltage = m_pdp->GetVoltage();           // Get the voltage of the PDP
    if (voltage != 0){
        double temperature = m_pdp->GetTemperature(); // Get the temperature of the PDP
        double current = m_pdp->GetCurrent(0);        // Get the current of channel 0
        double totalCurrent = m_pdp->GetTotalCurrent(); // Get the total current of all channels

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DragonPower", std::string("Voltage"), voltage);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DragonPower", std::string("Temperature"), temperature);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DragonPower", std::string("Current on Channel 0"), current);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DragonPower", std::string("Total Current"), totalCurrent);
    } else {
        //Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, "DragonPower", std::string("PDP not connected"));
    }
    

}



