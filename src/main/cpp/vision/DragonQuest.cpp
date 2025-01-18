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

DragonQuest::DragonQuest()
{
    m_networktable = nt::NetworkTableInstance::GetDefault().GetTable(std::string("questnav"));
}
DragonQuest *DragonQuest::m_dragonquest = nullptr;
DragonQuest *DragonQuest::GetDragonQuest()
{
    if (DragonQuest::m_dragonquest == nullptr)
    {
        DragonQuest::m_dragonquest = new DragonQuest();
    }
    return DragonQuest::m_dragonquest;
}
frc::Pose2d DragonQuest::GetEstimatedPose()
{
    auto postopic = m_networktable.get()->GetDoubleArrayTopic(std::string("position"));

    std::vector<double> posarray = postopic.GetEntry(std::array<double, 3>{}).Get();

    frc::Pose2d currentpos = frc::Pose2d{units::length::meter_t(posarray[0]), units::meter_t(posarray[1]), DragonQuest::GetOculusYaw()};
    return currentpos;
}

units::angle::degree_t DragonQuest::GetOculusYaw()
{
    auto rotationtopic = m_networktable.get()->GetDoubleArrayTopic(std::string("eulerAngles"));
    std::vector<double> rotationarray = rotationtopic.GetEntry(std::array<double, 3>{}).Get();
    double yaw = units::degree_t(rotationarray[2]).value() - yawoffset;
    if (yaw > 180)
    {
        yaw -= -360;
    }
    return units::angle::degree_t(yaw);
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
    auto rotationtopic = m_networktable.get()->GetDoubleArrayTopic(std::string("eulerAngles"));
    std::vector<double> rotationarray = rotationtopic.GetEntry(std::array<double, 3>{}).Get();
    yawoffset = rotationarray[2];
}

void DragonQuest::ZeroPosition()
{
}