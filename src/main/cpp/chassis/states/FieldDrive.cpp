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

#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"

// Team302 Includes
#include "chassis/states/FieldDrive.h"

/// DEBUGGING
#include "utils/logging/debug/Logger.h"

using frc::ChassisSpeeds;
using frc::Rotation2d;
using std::string;

FieldDrive::FieldDrive(RobotDrive *robotDrive) : RobotDrive(robotDrive->GetChassis()),
                                                 m_robotDrive(robotDrive)
{
}

std::array<frc::SwerveModuleState, 4> FieldDrive::UpdateSwerveModuleStates(ChassisMovement &chassisMovement)
{
    if (m_chassis != nullptr)
    {
        auto rot2d = Rotation2d(m_chassis->GetYaw());

        chassisMovement.chassisSpeeds = ChassisSpeeds::FromFieldRelativeSpeeds(chassisMovement.chassisSpeeds.vx,
                                                                               chassisMovement.chassisSpeeds.vy,
                                                                               chassisMovement.chassisSpeeds.omega,
                                                                               rot2d);
        chassisMovement.chassisSpeeds = frc::ChassisSpeeds::Discretize(chassisMovement.chassisSpeeds, units::time::millisecond_t(20.0));
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("FieldDrive"), string("chassis"), string("nullptr"));
    }

    return m_robotDrive->UpdateSwerveModuleStates(chassisMovement);
}

std::string FieldDrive::GetDriveStateName() const
{
    return std::string("FieldDrive");
}

void FieldDrive::Init(ChassisMovement &chassisMovement)
{
}
