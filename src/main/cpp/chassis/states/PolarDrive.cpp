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

#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Pose2d.h"

// Team302 Includes
#include "chassis/states/PolarDrive.h"

/// DEBUGGING
#include "utils/logging/Logger.h"
#include "utils/AngleUtils.h"

using frc::ChassisSpeeds;
using frc::Rotation2d;
using std::string;

PolarDrive::PolarDrive(RobotDrive *robotDrive) : RobotDrive(robotDrive->GetChassis()),
                                                 m_robotDrive(robotDrive)
{
    m_pid->EnableContinuousInput(-180.0, 180.0);
    m_pid->SetIZone(20.0);
    m_pid->SetIntegratorRange(-360.0, 360.0);
    m_pid->Reset();
}

std::array<frc::SwerveModuleState, 4> PolarDrive::UpdateSwerveModuleStates(ChassisMovement &chassisMovement)
{
    if (m_chassis != nullptr)
    {
        units::length::meter_t reefXPos = units::length::meter_t(4.5); // TODO needs updating based on alliance and use constants
        units::length::meter_t reefYPos = units::length::meter_t(4.0); // TODO needs updating based on alliance and use constants

        auto chassisSpeeds = chassisMovement.chassisSpeeds;

        frc::Pose2d currentPose = m_chassis->GetPose();
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "AlignDebugging", "Y", currentPose.Y().value());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "AlignDebugging", "X", currentPose.X().value());
        auto xDiff = currentPose.X() - reefXPos;
        auto yDiff = currentPose.Y() - reefYPos;

        double radius = units::math::hypot(xDiff, yDiff).value();
        double angle = (units::math::atan2(yDiff, xDiff)).value();

        // Radial velocity: Changes radius
        double radialVelocity = chassisSpeeds.vx.value(); // Forward/backward motion directly affects radius
        if (radialVelocity > 0.1)
        {
            radius += radialVelocity * m_loopRate * -1; // Negative since forward decreases radius
        }

        // Angular velocity: Changes angle
        double angularVelocity = chassisSpeeds.vy.value() / radius; // Clockwise/counter-clockwise motion
        angle += angularVelocity * m_loopRate;

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "AlignDebugging", "radialVelocity", radialVelocity);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "AlignDebugging", "angularVelocity", angularVelocity);

        // Convert polar velocities back to Cartesian
        double vxNew = radialVelocity * std::cos(angle) - (radius * angularVelocity * std::sin(angle));
        double vyNew = radialVelocity * std::sin(angle) + (radius * angularVelocity * std::cos(angle));

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "AlignDebugging", "Raidus", radius);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "AlignDebugging", "Angle", angle);

        chassisSpeeds.vx = units::velocity::meters_per_second_t(vxNew);
        chassisSpeeds.vy = units::velocity::meters_per_second_t(vyNew);
        auto rot2d = Rotation2d(m_chassis->GetYaw());

        // calculate the field relative angle
        units::angle::degree_t angleToReefCenter = units::angle::degree_t(units::math::atan2(currentPose.Y().value() - 4.0, currentPose.X().value() - 4.5));

        // wrap angleToReefCenter between -180 and 180 degrees
        angleToReefCenter = AngleUtils::GetEquivAngle(angleToReefCenter);

        units::angle::degree_t angleRelativeToFace = (units::math::fmod((angleToReefCenter + 30_deg), 60_deg)) - 30_deg;
        if (angleRelativeToFace < -30_deg)
        {
            angleRelativeToFace += 60_deg;
        }

        units::angle::degree_t closestMultiple = angleToReefCenter - angleRelativeToFace;
        units::angle::degree_t fieldRelativeAngle = AngleUtils::GetEquivAngle(closestMultiple);

        units::angle::degree_t currentAngle = currentPose.Rotation().Degrees();

        chassisMovement.chassisSpeeds.omega = units::angle::degree_t(m_pid->Calculate(currentAngle.value(), fieldRelativeAngle.value()));

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "AlignDebugging", "VyNew", vyNew);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "AlignDebugging", "VxNew", vxNew);
        chassisMovement.chassisSpeeds = ChassisSpeeds::FromFieldRelativeSpeeds(chassisMovement.chassisSpeeds.vx,
                                                                               chassisMovement.chassisSpeeds.vy,
                                                                               chassisMovement.chassisSpeeds.omega,
                                                                               rot2d);
        return m_robotDrive->UpdateSwerveModuleStates(chassisMovement);
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("PolarDrive"), string("chassis"), string("nullptr"));
    }

    return m_robotDrive->UpdateSwerveModuleStates(chassisMovement);
}

std::string PolarDrive::GetDriveStateName() const
{
    return std::string("PolarDrive");
}

void PolarDrive::Init(ChassisMovement &chassisMovement)
{
}
