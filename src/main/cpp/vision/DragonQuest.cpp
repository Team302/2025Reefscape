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
#include "vision/DragonQuest.h"

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
    m_limelightNetworktable = nt::NetworkTableInstance::GetDefault().GetTable(std::string("limelight-test"));
    m_questMosi = m_networktable.get()->GetIntegerTopic("mosi").Publish();
    m_questMiso = m_networktable.get()->GetIntegerTopic("miso").Subscribe(0);
    m_posTopic = m_networktable.get()->GetDoubleArrayTopic("position");
    m_rotationTopic = m_networktable.get()->GetDoubleArrayTopic("euler angles");
    ZeroHeading();
    ZeroPosition();
}

frc::Pose3d DragonQuest::GetEstimatedPose()
{
    DoStuff();
    if (m_loopcounter > 10)
    {
        ResetWithLimelightData();
    }
    std::vector<double> posarray = m_posTopic.GetEntry(std::array<double, 3>{}).Get();
    std::vector<double> rotationarray = m_rotationTopic.GetEntry(std::array<double, 3>{}).Get();

    double x = posarray[2] + m_xOffset;
    double y = posarray[0] + m_yOffset;
    double z = posarray[1] + m_zOffset;

    double roll = rotationarray[0] + m_rollOffset;
    double pitch = rotationarray[1] + m_pitchOffset;
    double yaw = rotationarray[2] + m_yawOffset;

    return frc::Pose3d{units::length::meter_t(x), units::length::meter_t(y), units::length::meter_t(z), frc::Rotation3d{units::angle::degree_t(roll), units::angle::degree_t(pitch), units::angle::degree_t(yaw)}};
}

units::angle::degree_t DragonQuest::GetOculusYaw()
{
    std::vector<double> rotationarray = m_rotationTopic.GetEntry(std::array<double, 3>{}).Get();
    m_yaw = rotationarray[1] - m_yawoffsetzero;
    if (m_yaw > 180)
    {
        m_yaw -= 360;
    }
    return units::angle::degree_t(m_yaw);
}

bool DragonQuest::IsConnected()
{
    return m_networktable.get()->GetBoolean(std::string("isconnected"), false);
}

double DragonQuest::GetBatteryPercent()
{
    return m_networktable.get()->GetNumber(std::string("batterypercent"), 0.0);
}

double DragonQuest::GetTimeStamp()
{
    return m_networktable.get()->GetNumber(std::string("timestamp"), 0.0);
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

void DragonQuest::DataLog()
{
    Log3DPoseData(DragonDataLoggerSignals::PoseSingals::CURRENT_CHASSIS_QUEST_POSE3D, GetEstimatedPose());
    std::vector<double> limelightpose = m_limelightPoseTopic.GetEntry(std::array<double, 10>{}).Get();
    Log3DPoseData(DragonDataLoggerSignals::PoseSingals::CURRENT_CHASSIS_LIMELIGHT_POSE3D, frc::Pose3d{units::length::meter_t(limelightpose[0]), units::length::meter_t(limelightpose[1]), units::length::meter_t(limelightpose[2]), frc::Rotation3d{units::angle::degree_t(limelightpose[3]), units::angle::degree_t(limelightpose[4]), units::angle::degree_t(limelightpose[5])}});
}

void DragonQuest::DoStuff()
{
    m_posTopic = m_networktable.get()->GetDoubleArrayTopic("position");
    m_rotationTopic = m_networktable.get()->GetDoubleArrayTopic("euler angles");
    m_limelightPoseTopic = m_limelightNetworktable.get()->GetDoubleArrayTopic("botpose_wpiblue");
    m_loopcounter++;
}

void DragonQuest::ResetWithLimelightData()
{
    std::vector<double> limelightpose = m_limelightPoseTopic.GetEntry(std::array<double, 10>{}).Get();
    m_xOffset + limelightpose[0];
    m_yOffset + limelightpose[1];
    m_zOffset + limelightpose[2];
    m_rollOffset + limelightpose[3];
    m_pitchOffset + limelightpose[4];
    m_yawOffset + limelightpose[5];
}