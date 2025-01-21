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
    std::vector<double> posarray = m_posTopic.GetEntry(std::array<double, 3>{}).Get();
    std::vector<double> rotationarray = m_rotationTopic.GetEntry(std::array<double, 3>{}).Get();

    return frc::Pose3d{units::length::meter_t(posarray[2]), units::length::meter_t(posarray[0]), units::length::meter_t(posarray[1]), frc::Rotation3d{units::angle::degree_t(rotationarray[0]), units::angle::degree_t(rotationarray[1]), units::angle::degree_t(rotationarray[2])}};
}

units::angle::degree_t DragonQuest::GetOculusYaw()
{
    std::vector<double> rotationarray = m_rotationTopic.GetEntry(std::array<double, 3>{}).Get();
    m_yaw = rotationarray[1] - m_yawoffset;
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
    m_yawoffset = rotationarray[2];
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
    Log3DPoseData(DragonDataLoggerSignals::PoseSingals::CURRENT_CHASSIS_POSE3D, GetEstimatedPose());
}

void DragonQuest::DoStuff()
{
    m_posTopic = m_networktable.get()->GetDoubleArrayTopic("position");
    m_rotationTopic = m_networktable.get()->GetDoubleArrayTopic("euler angles");
}