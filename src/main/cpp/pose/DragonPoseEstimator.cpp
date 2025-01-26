
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

#include "DragonPoseEstimator.h"

DragonPoseEstimator *DragonPoseEstimator::m_instance = nullptr;

DragonPoseEstimator *DragonPoseEstimator::GetInstance()
{
    if (DragonPoseEstimator::m_instance == nullptr)
    {
        DragonPoseEstimator::m_instance = new DragonPoseEstimator();
    }
    return DragonPoseEstimator::m_instance;
}

DragonPoseEstimator::DragonPoseEstimator() : m_vision(DragonVision::GetDragonVision()),
                                             m_chassis(),
                                             m_quest(DragonQuest::GetDragonQuest())

{
    auto chassisConfig = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    m_chassis = chassisConfig != nullptr ? chassisConfig->GetSwerveChassis() : nullptr;
}

frc::Pose3d DragonPoseEstimator::GetEstimatedPosition()
{
    GetVisonPose();
    GetChassisPose();
    GetQuestPose();
    if (m_vision != nullptr && m_chassis != nullptr && m_quest != nullptr)
    {

        double avgX = (m_visionPose.X().to<double>() * m_visionWeight) + (m_chassisPose.X().to<double>() * m_chassisWeight) + (m_QuestPose.X().to<double>() * m_questWeight);
        avgX /= 3;
        double avgY = (m_visionPose.Y().to<double>() * m_visionWeight) + (m_chassisPose.Y().to<double>() * m_chassisWeight) + (m_QuestPose.Y().to<double>() * m_questWeight);
        avgY /= 3;
        double avgZ = (m_visionPose.Z().to<double>() * m_visionWeight) + (m_QuestPose.Z().to<double>() * m_questWeight);
        avgZ /= 2;

        double avgRoll = (m_visionPose.Rotation().X().to<double>() * m_visionWeight) + (m_QuestPose.Rotation().X().to<double>() * m_questWeight);
        avgRoll /= 2;
        double avgPitch = (m_visionPose.Rotation().Y().to<double>() * m_visionWeight) + (m_QuestPose.Rotation().Y().to<double>() * m_questWeight);
        avgPitch /= 2;
        double avgYaw = (m_visionPose.Rotation().Z().to<double>() * m_visionWeight) + (m_chassisPose.Rotation().Z().to<double>() * m_chassisWeight) + (m_QuestPose.Rotation().Z().to<double>() * m_questWeight);
        avgYaw /= 3;

        return frc::Pose3d{units::length::meter_t(avgX), units::length::meter_t(avgY), units::length::meter_t(avgZ), frc::Rotation3d{units::angle::degree_t(avgRoll), units::angle::degree_t(avgPitch), units::angle::degree_t(avgYaw)}};
    }
}

frc::Pose3d DragonPoseEstimator::GetVisonPose()
{
    if (m_vision != nullptr)
    {
        auto megatag1 = m_vision->GetRobotPosition();
        if (megatag1.has_value())
        {
            m_visionPose = megatag1.value().estimatedPose;
        }
    }
}

frc::Pose2d DragonPoseEstimator::GetChassisPose()
{
    if (m_chassis != nullptr)
    {
        return m_chassis->GetPose();
    }
}

frc::Pose3d DragonPoseEstimator::GetQuestPose()
{
    if (m_quest != nullptr)
    {
        return m_quest->GetEstimatedPose();
    }
}
