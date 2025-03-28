
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
#include "utils/logging/debug/Logger.h"
#include "frc/Filesystem.h"

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
                             m_fieldConstants(FieldConstants::GetInstance())
{
    CalculateZones();
    InitZones();
}

void BargeHelper::CalculateZones()
{
    auto allianceColor = FMSData::GetInstance()->GetAllianceColor();
    bool isRed = allianceColor == frc::DriverStation::Alliance::kRed;
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

void BargeHelper::InitZones()
{

    m_bargeZonesRed = ZoneParser::ParseXML("RedBarge.xml");
    m_bargeZonesBlue = ZoneParser::ParseXML("BlueBarge.xml");
}
std::optional<units::length::meter_t> BargeHelper::ClampChassisY()
{
    auto allianceColor = FMSData::GetInstance()->GetAllianceColor();
    auto bargeZones = allianceColor == frc::DriverStation::Alliance::kRed ? m_bargeZonesRed : m_bargeZonesBlue;
    if (bargeZones != nullptr)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Barge Helper", "Parsed Y1", bargeZones->GetY1Rect().value());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Barge Helper", "Parsed Y2", bargeZones->GetY2Rect().value());
        auto min = std::min(bargeZones->GetY2Rect(), bargeZones->GetY1Rect());
        auto max = std::max(bargeZones->GetY2Rect(), bargeZones->GetY1Rect());
        return std::clamp(m_chassis->GetPose().Y(), min, max);
    }

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Barge Helper", "Barge Zone", "nullptr");
    return std::nullopt;
}

void BargeHelper::IsInZone()
{
    auto allianceColor = FMSData::GetInstance()->GetAllianceColor();
    auto bargeZones = allianceColor == frc::DriverStation::Alliance::kRed ? m_bargeZonesRed : m_bargeZonesBlue;

    if (bargeZones != nullptr)
    {
        bool intheZone = AutonGrid::GetInstance()->IsPoseInZone(bargeZones->GetX1Rect(), bargeZones->GetX2Rect(), bargeZones->GetY1Rect(), bargeZones->GetY2Rect(), m_chassis->GetPose());
        if (m_previousIsInZone != intheZone)
            RobotState::GetInstance()->PublishStateChange(RobotStateChanges::StateChange::IsInBargeZone_Bool, intheZone);
        m_previousIsInZone = intheZone;
    }
}

frc::Pose2d BargeHelper::CalcBargePose()
{
    auto allianceColor = FMSData::GetInstance()->GetAllianceColor();
    bool inBargeHalf = false;
    bool pastReef = false;
    bool behindFrontFace = false;
    bool clearOfReef = false;
    frc::Pose2d pose2d{};
    if (allianceColor == frc::DriverStation::Alliance::kRed)
    {
        pose2d = m_chassis->GetPose().X() > m_centerLine ? m_fieldConstants->GetFieldElementPose(FieldConstants::FIELD_ELEMENT::RED_BARGE_FRONT_CALCULATED).ToPose2d() : m_fieldConstants->GetFieldElementPose(FieldConstants::FIELD_ELEMENT::RED_BARGE_BACK_CALCULATED).ToPose2d();
        inBargeHalf = m_chassis->GetPose().Y() < m_fieldConstants->GetAprilTagPose(FieldConstants::AprilTagIDs::RED_REEF_GH_TAG).ToPose2d().Y();
        pastReef = m_chassis->GetPose().X() < m_fieldConstants->GetAprilTagPose(FieldConstants::AprilTagIDs::RED_REEF_GH_TAG).ToPose2d().X();
        behindFrontFace = m_chassis->GetPose().X() > m_fieldConstants->GetFieldElementPose(FieldConstants::FIELD_ELEMENT::RED_REEF_AB).ToPose2d().X();
        clearOfReef = m_chassis->GetPose().Y() < 2.0_m;
        m_avoidReefY = behindFrontFace ? 2.0_m : 6.0_m;
    }
    else
    {
        pose2d = m_chassis->GetPose().X() > m_centerLine ? m_fieldConstants->GetFieldElementPose(FieldConstants::FIELD_ELEMENT::BLUE_BARGE_BACK_CALCULATED).ToPose2d() : m_fieldConstants->GetFieldElementPose(FieldConstants::FIELD_ELEMENT::BLUE_BARGE_FRONT_CALCULATED).ToPose2d();
        inBargeHalf = m_chassis->GetPose().Y() > m_fieldConstants->GetAprilTagPose(FieldConstants::AprilTagIDs::BLUE_REEF_GH_TAG).ToPose2d().Y();
        pastReef = m_chassis->GetPose().X() > m_fieldConstants->GetAprilTagPose(FieldConstants::AprilTagIDs::BLUE_REEF_GH_TAG).ToPose2d().X();
        behindFrontFace = m_chassis->GetPose().X() < m_fieldConstants->GetFieldElementPose(FieldConstants::FIELD_ELEMENT::BLUE_REEF_AB).ToPose2d().X();
        clearOfReef = m_chassis->GetPose().Y() > 6.0_m;
        m_avoidReefY = behindFrontFace ? 6.0_m : 2.0_m;
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Barge Helper", "In Barge Half", inBargeHalf);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Barge Helper", "Past Reef", pastReef);

    if (behindFrontFace && m_chassis->GetPose().Y())
    {
        pose2d = frc::Pose2d(m_chassis->GetPose().X(), m_avoidReefY, pose2d.Rotation());
    }
    else if (!inBargeHalf && !pastReef)
    {
        pose2d = frc::Pose2d(pose2d.X(), m_avoidReefY, pose2d.Rotation());
    }
    else
    {
        if (ClampChassisY().has_value())
        {
            pose2d = frc::Pose2d(pose2d.X(), ClampChassisY().value(), pose2d.Rotation());
        }
        else
        {
            pose2d = frc::Pose2d(pose2d.X(), m_chassis->GetPose().Y(), pose2d.Rotation());
        }
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Barge Helper", "Pose Y", pose2d.Y().value());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Barge Helper", "Avoid Reef Y", m_avoidReefY.value());

    return pose2d;
}

frc::Pose2d BargeHelper::GetCagePose(DragonTargetFinderTarget target)
{
    auto allianceColor = FMSData::GetInstance()->GetAllianceColor();
    auto fieldElement = allianceColor == frc::DriverStation::kRed ? FieldConstants::FIELD_ELEMENT::RED_BARGE_FRONT_CALCULATED : FieldConstants::FIELD_ELEMENT::BLUE_BARGE_FRONT_CALCULATED; // defualt value in front of barge

    if (target == DragonTargetFinderTarget::LEFT_CAGE)
        fieldElement = allianceColor == frc::DriverStation::kRed ? FieldConstants::FIELD_ELEMENT::RED_LEFT_CAGE : FieldConstants::FIELD_ELEMENT::BLUE_LEFT_CAGE;
    else if (target == DragonTargetFinderTarget::RIGHT_CAGE)
        fieldElement = allianceColor == frc::DriverStation::kRed ? FieldConstants::FIELD_ELEMENT::RED_RIGHT_CAGE : FieldConstants::FIELD_ELEMENT::BLUE_RIGHT_CAGE;
    else if (target == DragonTargetFinderTarget::CENTER_CAGE)
        fieldElement = allianceColor == frc::DriverStation::kRed ? FieldConstants::FIELD_ELEMENT::RED_CENTER_CAGE : FieldConstants::FIELD_ELEMENT::BLUE_CENTER_CAGE;

    return m_fieldConstants->GetFieldElementPose(fieldElement).ToPose2d();
}
