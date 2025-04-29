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

#include <string>

// FRC Includes
#include "frc/Timer.h"

// Team302 Includes
#include "chassis/states/RobotDrive.h"
#include "chassis/SwerveChassis.h"
#include "fielddata/DragonTargetFinder.h"

// Third party includes

class TrajectoryDrive : public RobotDrive
{
public:
    TrajectoryDrive(RobotDrive *robotDrive);
    std::string GetDriveStateName() const override;

    std::array<frc::SwerveModuleState, 4> UpdateSwerveModuleStates(ChassisMovement &chassisMovement) override;

    void Init(ChassisMovement &chassisMovement) override;

    std::string WhyDone() const { return m_whyDone; };
    virtual bool IsDone();

    units::time::second_t GetTotalTrajectoryTime() const { return m_totalTrajectoryTime; }

protected:
    const units::meters_per_second_t m_maxVel = 1_mps;
    const units::meters_per_second_squared_t m_maxAccel = 0.5_mps_sq;
    const units::radians_per_second_t m_maxAngularVel = 540_deg_per_s;
    const units::radians_per_second_squared_t m_maxAngularAccel = 720_deg_per_s_sq;

private:
    void LogMoveInfo(ChassisMovement &moveInfo);
    bool IsSamePose(frc::Pose2d currentPose, frc::Pose2d previousPose, frc::ChassisSpeeds velocity, double xyTolerance, double rotTolerance, double speedTolerance);

    void LogPose(frc::Pose2d pose) const;

    std::optional<choreo::Trajectory<choreo::SwerveSample>> m_trajectory;
    choreo::SwerveSample m_finalState;
    std::vector<choreo::SwerveSample> m_trajectoryStates;

    RobotDrive *m_robotDrive;
    frc::Pose2d m_prevPose;
    bool m_wasMoving;
    frc::Transform2d m_delta;
    std::unique_ptr<frc::Timer> m_timer;

    std::string m_whyDone;
    units::time::second_t m_totalTrajectoryTime;

    double m_kPCoarse = 5.0;
    double m_kPFine = 9.0;
    const double m_percentageCompleteThreshold = 0.90;
    int m_samePoseCount = 0;
    const int m_samePoseCountThreshold = 50; // TODO come back and tune this

    frc::PIDController xController{10.0, 0.0, 0.0};
    frc::PIDController yController{10.0, 0.0, 0.0};
    frc::PIDController headingController{7.5, 0.0, 0.0};
};