
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

#include "chassis/definitions/ChassisConfig.h"
#include "chassis/definitions/ChassisConfigMgr.h"
#include "fielddata/BargeHelper.h"
#include "frc/DriverStation.h"
#include "utils/FMSData.h"

BargeHelper *BargeHelper::m_instance = nullptr;
BargeHelper *BargeHelper::GetInstance()
{
    if (BargeHelper::m_instance == nullptr)
    {
        BargeHelper::m_instance = new BargeHelper();
    }
    return BargeHelper::m_instance;
}

BargeHelper::BargeHelper() : m_chassis(ChassisConfigMgr::GetInstance()->GetCurrentChassis()),
                             m_allianceColor(FMSData::GetInstance()->GetAllianceColor()),
                             m_fieldConstants(FieldConstants::GetInstance())
{
    CalculateZones();
}

void BargeHelper::CalculateZones()
{
    bool isRed = m_allianceColor == frc::DriverStation::Alliance::kRed;
    auto sizeOfBarge = 0_m;

    sizeOfBarge = isRed ? m_redLeftBargePose.Translation().Distance(m_redRightBargePose.Translation()) : m_blueLeftBargePose.Translation().Distance(m_blueRightBargePose.Translation());

    auto sizeOfZones = sizeOfBarge / m_numOfZones;
    for (unsigned int i = 0; i < m_numOfZones; i++)
    {
        if (isRed)
        {
            m_zonesVector.emplace_back(frc::Pose2d(m_redLeftBargePose.X(), units::length::meter_t(m_redLeftBargePose.Y() + (sizeOfZones * i)), 0_deg));
        }
        else
        {
            m_zonesVector.emplace_back(frc::Pose2d(m_blueRightBargePose.X(), units::length::meter_t(m_blueRightBargePose.Y() + (sizeOfZones * i)), 0_deg));
        }
    }
}

std::optional<BargeZones> BargeHelper::GetClosestZone()
{
    BargeZones closestZone = BargeZones::NO_ZONE;
    if (!m_zonesVector.empty())
    {
        auto closestTranslation = m_zonesVector[0].Translation().Distance(m_chassis->GetPose().Translation());
        for (unsigned int i = 0; i < m_zonesVector.size(); i++)
        {
            auto currentZoneDistance = m_zonesVector[i].Translation().Distance(m_chassis->GetPose().Translation());
            if (closestTranslation > currentZoneDistance)
            {
                closestTranslation = currentZoneDistance;
                closestZone = static_cast<BargeZones>(i);
            }
        }
        return closestZone;
    }
    return std::nullopt;
}
