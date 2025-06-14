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

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Pose2d.h>
#include "chassis/generated/CommandSwerveDrivetrain.h"
#include "fielddata/DragonTargetFinder.h"

class DriveToTarget : public frc2::CommandHelper<frc2::Command, DriveToTarget>
{
public:
    /**
     * @brief Creates a command to drive to a specified field element using vision and odometry.
     *
     * @param chassis A pointer to the swerve drive subsystem.
     * @param target The specific field element to target.
     */
    DriveToTarget(subsystems::CommandSwerveDrivetrain *chassis, DragonTargetFinderTarget target);

    // FRC Command Lifecycle methods
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

private:
    void CalculateFeedForward(units::meters_per_second_t &vx, units::meters_per_second_t &vy);

    subsystems::CommandSwerveDrivetrain *m_chassis;
    DragonTargetFinderTarget m_target;
    DragonTargetFinder *m_targetFinder;

    swerve::requests::RobotCentricFacingAngle m_driveRequest;

    // State variables
    bool m_hasTarget;
    frc::Pose2d m_targetPose;
    int m_samePoseCount;

    // PID controllers for translation correction. Gains will need tuning.
    frc::ProfiledPIDController<units::meters> m_translationPIDX;
    frc::ProfiledPIDController<units::meters> m_translationPIDY;

    // Constants from your old DriveToFieldElement class
    const units::length::inch_t m_distanceThreshold{0.25};
    const units::length::meter_t m_ffMinRadius{0.0};
    const units::length::meter_t m_ffMaxRadius{1.25};
    const units::velocity::meters_per_second_t kMaxVelocity = 4_mps;
    const units::acceleration::meters_per_second_squared_t kMaxAcceleration = 4_mps_sq;
};
