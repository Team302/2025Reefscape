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
#include "units/time.h"
#include "utils/AngleUtils.h"
#include "vision/DragonQuest.h"
#include "utils/DragonField.h"

DragonQuest *DragonQuest::m_dragonquest = nullptr;
DragonQuest *DragonQuest::GetDragonQuest()
{
    if (DragonQuest::m_dragonquest == nullptr)
    {
        return nullptr;
    }
    return DragonQuest::m_dragonquest;
}

DragonQuest::DragonQuest(
    units::length::inch_t mountingXOffset, /// <I> x offset of cam from robot center (forward relative to robot)
    units::length::inch_t mountingYOffset, /// <I> y offset of cam from robot center (left relative to robot)
    units::length::inch_t mountingZOffset, /// <I> z offset of cam from robot center (up relative to robot)
    units::angle::degree_t mountingPitch,  /// <I> - Pitch of camera
    units::angle::degree_t mountingYaw,    /// <I> - Yaw of camera
    units::angle::degree_t mountingRoll    /// <I> - Roll of camera
    ) : m_mountingXOffset(mountingXOffset),
        m_mountingYOffset(mountingYOffset),
        m_mountingZOffset(mountingZOffset),
        m_mountingPitch(mountingPitch),
        m_mountingYaw(mountingYaw),
        m_mountingRoll(mountingRoll)
{
    m_networktable = nt::NetworkTableInstance::GetDefault().GetTable(std::string("questnav"));
    m_questMosi = m_networktable.get()->GetIntegerTopic("mosi").Publish();
    m_questMiso = m_networktable.get()->GetIntegerTopic("miso").Subscribe(0);
    m_posTopic = m_networktable.get()->GetDoubleArrayTopic("position");
    m_rotationTopic = m_networktable.get()->GetDoubleArrayTopic("euler angles");
    m_initialPosePublisher = m_networktable.get()->GetDoubleArrayTopic("resetpose").Publish();

    // ZeroPosition();
}

frc::Pose2d DragonQuest::GetEstimatedPose()
{
    RefreshNT();

    std::vector<double> posarray = m_posTopic.GetEntry(std::array<double, 3>{}).Get();
    std::vector<double> rotationarray = m_rotationTopic.GetEntry(std::array<double, 3>{}).Get();

    units::length::meter_t x{posarray[2]};
    units::length::meter_t y{-posarray[0]};
    units::length::meter_t z{posarray[1]};
    units::angle::degree_t roll{rotationarray[0]};
    units::angle::degree_t pitch{rotationarray[2]};
    units::angle::degree_t yaw{-rotationarray[1]};

    frc::Pose2d questPose{x, y, yaw};
    frc::Pose2d robotPose = questPose + m_questTransform;
    return robotPose;
}

bool DragonQuest::IsConnected()
{
    if (m_posTopic.GetEntry(std::array<double, 3>{}).Get()[0] != 0)
    {
        return true;
    }
    return false;
}

void DragonQuest::ZeroPosition()
{
    if (m_questMiso.Get() != 99)
    {
        m_questMosi.Set(1);
    }
}

void DragonQuest::DataLog(uint64_t timestamp)
{
    Log2DPoseData(timestamp, DragonDataLoggerSignals::PoseSingals::CURRENT_CHASSIS_QUEST_POSE2D, GetEstimatedPose());
    auto field = DragonField::GetInstance();
    field->AddPose("Quest", GetEstimatedPose());
}

void DragonQuest::RefreshNT()
{
    m_posTopic = m_networktable.get()->GetDoubleArrayTopic("position");
    m_rotationTopic = m_networktable.get()->GetDoubleArrayTopic("eulerAngles");
}

void DragonQuest::SetRobotPose(const frc::Pose2d &pose)
{
    if (!m_hasreset)
    {
        // Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonQuest"), string("SetRobotPose"), string("not hasreset"));
        // Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonQuest"), string("x"), pose.X().value());
        // Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonQuest"), string("y"), pose.Y().value());
        // Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonQuest"), string("rot"), pose.Rotation().Degrees().value());
        auto x = (pose.X() + m_mountingXOffset);
        auto y = (pose.Y() + m_mountingYOffset);
        auto rot = (pose.Rotation().Degrees() + m_mountingYaw);

        m_initialPosePublisher.Set(std::array<double, 3>{x.value(), y.value(), rot.value()});

        m_questTransform = pose - frc::Pose2d{x, y, frc::Rotation2d(rot)};

        if (m_questMiso.Get() != 99)
        {
            sleep(1);
            m_questMosi.Set(2);
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonQuest"), string("SetRobotPose"), string("Resetting Pose"));
        }
        m_hasreset = true;
    }
}

DragonVisionPoseEstimatorStruct DragonQuest::GetPoseEstimate()
{
    DragonVisionPoseEstimatorStruct str;
    if (!m_hasreset || !IsConnected())
    {
        str.m_confidenceLevel = DragonVisionPoseEstimatorStruct::ConfidenceLevel::NONE;
    }
    else
    {
        str.m_confidenceLevel = DragonVisionPoseEstimatorStruct::ConfidenceLevel::NONE;
        str.m_visionPose = GetEstimatedPose();
        str.m_stds = wpi::array{m_stdxy, m_stdxy, m_stddeg};
    }
    return str;
}
