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
#include "vision/DragonQuest.h"
#include "utils/AngleUtils.h"

DragonQuest *DragonQuest::m_dragonquest = nullptr;
DragonQuest *DragonQuest::GetDragonQuest()
{
    if (DragonQuest::m_dragonquest == nullptr)
    {
        DragonQuest::m_dragonquest = new DragonQuest();
    }
    return DragonQuest::m_dragonquest;
}

DragonQuest::DragonQuest()
{
    m_networktable = nt::NetworkTableInstance::GetDefault().GetTable(std::string("questnav"));
    m_questMosi = m_networktable.get()->GetIntegerTopic("mosi").Publish();
    m_questMiso = m_networktable.get()->GetIntegerTopic("miso").Subscribe(0);
    m_posTopic = m_networktable.get()->GetDoubleArrayTopic("position");
    m_rotationTopic = m_networktable.get()->GetDoubleArrayTopic("euler angles");
    ZeroHeading();
    ZeroPosition();
}

frc::Pose3d DragonQuest::GetEstimatedPose()
{
    RefreshNT();
    std::vector<double> posarray = m_posTopic.GetEntry(std::array<double, 3>{}).Get();
    std::vector<double> rotationarray = m_rotationTopic.GetEntry(std::array<double, 3>{}).Get();

    double x = posarray[0] + m_xOffset;
    double y = posarray[2] + m_yOffset;
    double z = posarray[1] + m_zOffset;

    double roll = rotationarray[0] + m_rollOffset;
    double pitch = rotationarray[2] + m_pitchOffset;
    double yaw = rotationarray[1] + m_yawOffset;

    return frc::Pose3d{units::length::meter_t(x), units::length::meter_t(y), units::length::meter_t(z), frc::Rotation3d{units::angle::degree_t(roll), units::angle::degree_t(pitch), units::angle::degree_t(yaw)}};
}

bool DragonQuest::IsConnected()
{
    if (m_posTopic.GetEntry(std::array<double, 3>{}).Get()[0] != 0)
    {
        return true;
    }
    return false;
}

void DragonQuest::ZeroHeading()
{
    std::vector<double> rotationarray = m_rotationTopic.GetEntry(std::array<double, 3>{}).Get();
    m_yawoffsetzero = rotationarray[2];
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
    Log3DPoseData(timestamp, DragonDataLoggerSignals::PoseSingals::CURRENT_CHASSIS_QUEST_POSE3D, GetEstimatedPose());
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
        frc::Pose3d p3d{frc::Pose3d(pose)};
        m_xOffset += p3d.X().to<double>();
        m_yOffset += p3d.Y().to<double>();
        m_zOffset += p3d.Z().to<double>();
        units::degree_t roll = p3d.Rotation().X();
        units::degree_t pitch = p3d.Rotation().Y();
        units::degree_t yaw = p3d.Rotation().Z();

        m_rollOffset += roll.to<double>();
        m_pitchOffset += pitch.to<double>();
        m_yawOffset += yaw.to<double>();
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
        str.m_visionPose = GetEstimatedPose().ToPose2d();
        str.m_stds = wpi::array{m_stdxy, m_stdxy, m_stddeg};
    }
    return str;
}
