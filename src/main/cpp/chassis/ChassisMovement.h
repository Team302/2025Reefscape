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

#pragma once

// FRC Includes
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/trajectory/TrajectoryUtil.h"

#include "units/angle.h"

// Team302 Includes
#include "chassis/ChassisOptionEnums.h"

// Third party includes
#include "pathplanner/lib/trajectory/PathPlannerTrajectory.h"

/// @brief This is used to give all neccessary data to ISwerveDriveStates

struct ChassisMovement
{
    double rawX;
    double rawY;
    double rawOmega;
    ChassisOptionEnums::DriveStateType driveOption = ChassisOptionEnums::DriveStateType::ROBOT_DRIVE;
    ChassisOptionEnums::DriveStateType previousDriveOption = ChassisOptionEnums::DriveStateType::ROBOT_DRIVE;
    frc::ChassisSpeeds chassisSpeeds = frc::ChassisSpeeds();
    pathplanner::PathPlannerTrajectory pathplannerTrajectory = pathplanner::PathPlannerTrajectory();
    ChassisOptionEnums::PathGainsType pathnamegains = ChassisOptionEnums::PathGainsType::LONG;
    frc::Translation2d centerOfRotationOffset = frc::Translation2d();
    ChassisOptionEnums::HeadingOption headingOption = ChassisOptionEnums::HeadingOption::MAINTAIN;
    ChassisOptionEnums::NoMovementOption noMovementOption = ChassisOptionEnums::NoMovementOption::STOP;
    ChassisOptionEnums::AutonControllerType controllerType = ChassisOptionEnums::AutonControllerType::RAMSETE;
    units::angle::degree_t yawAngle = units::angle::degree_t(0.0);
    bool checkTipping = false;
    units::angle::degree_t tippingTolerance = units::angle::degree_t(5.0);
    double tippingCorrection = -0.1;
    frc::Pose2d targetPose = frc::Pose2d();
};