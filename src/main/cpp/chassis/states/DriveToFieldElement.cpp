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
//=====================================================================================================================================================

// C++ Includes

// FRC Includes
#include <frc/geometry/Pose2d.h>
#include <pathplanner/lib/path/PathConstraints.h>
#include <pathplanner/lib/path/PathPlannerPath.h>

// Team302 Includes
#include "chassis/definitions/ChassisConfig.h"
#include "chassis/definitions/ChassisConfigMgr.h"
#include "chassis/states/DriveToFieldElement.h"
#include "chassis/SwerveChassis.h"
#include "vision/DragonVisionStructs.h"
#include "vision/DragonVisionStructLogger.h"
#include "fielddata/DragonTargetFinder.h"
#include "utils/AngleUtils.h"

#include "utils/logging/debug/Logger.h"
#include "utils/logging/debug/LoggerData.h"
#include "utils/logging/debug/LoggerEnums.h"

using namespace pathplanner;
using namespace std;

DriveToFieldElement::DriveToFieldElement(RobotDrive *robotDrive) : RobotDrive(robotDrive->GetChassis()),
                                                                   m_robotDrive(robotDrive)
{
}

void DriveToFieldElement::Init(ChassisMovement &chassisMovement)
{
    InitChassisMovement(chassisMovement);
    auto info = DragonTargetFinder::GetInstance()->GetPose(GetDriveToTarget());
    m_currentType = get<0>(info.value());
    m_endPose = get<1>(info.value());

    if (m_chassis != nullptr)
    {
        m_translationPIDX.Reset(m_chassis->GetPose().X(), chassisMovement.chassisSpeeds.vx);
        m_translationPIDY.Reset(m_chassis->GetPose().Y(), chassisMovement.chassisSpeeds.vy);
    }
    CalculateFeedForward(chassisMovement);
}

std::array<frc::SwerveModuleState, 4> DriveToFieldElement::UpdateSwerveModuleStates(ChassisMovement &chassisMovement)
{
    /** TODO: TEMPORARY
    if (m_chassis != nullptr)
    {
        CalculateFeedForward(chassisMovement);
        auto chassisSpeeds = chassisMovement.chassisSpeeds;
        frc::Pose2d currentPose = m_chassis->GetPose();

        auto info = DragonTargetFinder::GetInstance()->GetPose(GetDriveToTarget());
        if (info.has_value())
        {
            frc::Pose2d newEndPose = get<1>(info.value());
            auto regenerate = false;

            regenerate = m_endPose.Translation().Distance(newEndPose.Translation()) > m_distanceThreshold;

            if ((m_currentType == DragonTargetFinderData::ODOMETRY_BASED) && (get<0>(info.value()) == DragonTargetFinderData::VISION_BASED) && regenerate) // If we are in odometry but get vision based pose regenerate
            {
                m_endPose = newEndPose;
            }
            m_currentType = get<0>(info.value());
        }

        DragonVisionStructLogger::logPose2d("current pose", currentPose);
        DragonVisionStructLogger::logPose2d("target pose", m_endPose);

        m_translationPIDX.SetGoal(m_endPose.X());
        m_translationPIDY.SetGoal(m_endPose.Y());

        chassisSpeeds.vx += units::velocity::meters_per_second_t(m_translationPIDX.Calculate(currentPose.X(), m_endPose.X()));
        chassisSpeeds.vy += units::velocity::meters_per_second_t(m_translationPIDY.Calculate(currentPose.Y(), m_endPose.Y()));

        chassisSpeeds.vx = std::clamp(chassisSpeeds.vx, -kMaxVelocity, kMaxVelocity);
        chassisSpeeds.vy = std::clamp(chassisSpeeds.vy, -kMaxVelocity, kMaxVelocity);

        units::angle::degree_t rotationError = chassisMovement.yawAngle - currentPose.Rotation().Degrees();
        rotationError = AngleUtils::GetEquivAngle(rotationError);
        chassisSpeeds.omega = std::clamp(units::angular_velocity::degrees_per_second_t(m_rotationKP * rotationError.value()), -kMaxAngularVelocity, kMaxAngularVelocity);

        auto rot2d = frc::Rotation2d(m_chassis->GetYaw());
        chassisMovement.chassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(chassisSpeeds.vx,
                                                                                    chassisSpeeds.vy,
                                                                                    chassisSpeeds.omega,
                                                                                    rot2d);
    }
    **/
    // TODO: TEMPORARY
    chassisMovement.chassisSpeeds.vx = units::velocity::meters_per_second_t(0.0);
    chassisMovement.chassisSpeeds.vy = units::velocity::meters_per_second_t(0.0);
    chassisMovement.chassisSpeeds.omega = units::angular_velocity::turns_per_second_t(0.0);
    return m_robotDrive->UpdateSwerveModuleStates(chassisMovement);
}

void DriveToFieldElement::InitChassisMovement(ChassisMovement &chassisMovement)
{
    // initialize the same as holonomic drive
    chassisMovement.rawOmega = 0.0;
    chassisMovement.chassisSpeeds.vx = units::velocity::meters_per_second_t(0.0);
    chassisMovement.chassisSpeeds.vy = units::velocity::meters_per_second_t(0.0);
    chassisMovement.driveOption = GetDriveStateType();
    chassisMovement.controllerType = ChassisOptionEnums::AutonControllerType::HOLONOMIC;
    chassisMovement.headingOption = GetHeadingOption();
    chassisMovement.pathplannerTrajectory = pathplanner::PathPlannerTrajectory();
    chassisMovement.centerOfRotationOffset = frc::Translation2d();
    chassisMovement.noMovementOption = ChassisOptionEnums::NoMovementOption::STOP;
    chassisMovement.pathnamegains = ChassisOptionEnums::PathGainsType::LONG;
    chassisMovement.chassisSpeeds.omega = units::angular_velocity::radians_per_second_t(0);
    chassisMovement.checkTipping = false;
    chassisMovement.tippingTolerance = units::angle::degree_t(5.0);
    chassisMovement.tippingCorrection = -0.1;
    chassisMovement.targetPose = frc::Pose2d();
}
void DriveToFieldElement::LogMoveInfo(ChassisMovement &moveInfo)
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToFieldElement", "heading option", moveInfo.headingOption);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToFieldElement", "drive option", moveInfo.driveOption);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToFieldElement", "vx", moveInfo.chassisSpeeds.vx.value());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToFieldElement", "vy", moveInfo.chassisSpeeds.vy.value());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToFieldElement", "omega", moveInfo.chassisSpeeds.omega.value());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToFieldElement", "target pose x", moveInfo.targetPose.X().value());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToFieldElement", "target pose y", moveInfo.targetPose.Y().value());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToFieldElement", "Target Pose Rotation", moveInfo.targetPose.Rotation().Degrees().value());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToFieldElement", "yaw angle", moveInfo.yawAngle.value());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToFieldElement", "raw omega", moveInfo.rawOmega);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToFieldElement", "controller type", moveInfo.controllerType);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToFieldElement", "no movement option", moveInfo.noMovementOption);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToFieldElement", "Target Pose X", m_endPose.X().value());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DriveToFieldElement", "Target Pose Y", m_endPose.Y().value());
}

bool DriveToFieldElement::IsDone()
{
    if (m_chassis != nullptr)
    {
        auto currentPose = m_chassis->GetPose();
        auto distance = currentPose.Translation().Distance(m_endPose.Translation());
        return (distance < m_distanceThreshold);
    }
    return true;
}

void DriveToFieldElement::CalculateFeedForward(ChassisMovement &chassisMovement)
{
    if (m_chassis != nullptr)
    {
        frc::Pose2d currentPose = m_chassis->GetPose();
        units::meter_t distance = currentPose.Translation().Distance(m_endPose.Translation());

        // Calculate feedforward speed based on distance
        units::velocity::meters_per_second_t feedforwardSpeed = 0.0_mps;
        if (distance > m_ffMinRadius)
        {
            double feedForwardScale = std::clamp(((distance - m_ffMinRadius) / (m_ffMaxRadius - m_ffMinRadius)).value(), 0.0, 1.0);
            feedforwardSpeed = kMaxVelocity * feedForwardScale;
        }

        // Apply feedforward to the desired velocity
        frc::Translation2d translationError = m_endPose.Translation() - currentPose.Translation();
        frc::Rotation2d angleToTarget = translationError.Angle();

        chassisMovement.chassisSpeeds.vx = feedforwardSpeed * angleToTarget.Cos();
        chassisMovement.chassisSpeeds.vy = feedforwardSpeed * angleToTarget.Sin();
    }
}