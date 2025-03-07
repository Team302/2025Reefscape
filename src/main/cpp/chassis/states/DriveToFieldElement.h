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

// C++
#include <optional>
#include <tuple>
#include <vector>

// FRC Includes
#include "frc/geometry/Pose2d.h"
#include "units/length.h"
#include "units/angle.h"
#include "units/velocity.h"
#include "units/angular_velocity.h"
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>

// Team302 Includes
#include "chassis/states/RobotDrive.h"
#include "chassis/ChassisOptionEnums.h"
#include "fielddata/DragonTargetFinder.h"

class DriveToFieldElement : public RobotDrive
{
public:
    DriveToFieldElement(RobotDrive *robotDrive);

    std::array<frc::SwerveModuleState, 4> UpdateSwerveModuleStates(ChassisMovement &chassisMovement) override;

    void Init(ChassisMovement &chassisMovement) override;
    bool IsDone();

protected:
    virtual DragonTargetFinderTarget GetDriveToTarget() const = 0;
    virtual ChassisOptionEnums::DriveStateType GetDriveStateType() const = 0;
    virtual ChassisOptionEnums::HeadingOption GetHeadingOption() const = 0;
    virtual units::angle::degree_t GetModifiedHeadingValue(units::angle::degree_t calculatedHeading) { return (calculatedHeading - 180_deg); }

private:
    void LogMoveInfo(ChassisMovement &moveInfo);

    RobotDrive *m_robotDrive;

    void InitChassisMovement(ChassisMovement &chassisMovement);
    void CalculateFeedForward(ChassisMovement &chassisMovement);

    DragonTargetFinderData m_currentType = DragonTargetFinderData::NOT_FOUND;
    frc::Pose2d m_endPose;
    const unsigned int m_generatedStatesThreshold = 1;

    const units::length::inch_t m_distanceThreshold{0.25};
    const units::length::meter_t m_ffMinRadius{0.25};
    const units::length::meter_t m_ffMaxRadius{1.0};

    const units::velocity::meters_per_second_t kMaxVelocity = 3_mps;
    const units::acceleration::meters_per_second_squared_t kMaxAcceleration = 1_mps_sq;

    const units::angular_velocity::degrees_per_second_t kMaxAngularVelocity = 540_deg_per_s;

    const double m_translationKP = 4.0;
    const double m_translationKI = 0.5;
    const double m_translationKD = 0.2;

    const double m_rotationKP = 6.0;

    frc::TrapezoidProfile<units::length::meters>::Constraints m_translationConstraints{kMaxVelocity, kMaxAcceleration};

    frc::ProfiledPIDController<units::length::meters> m_translationPIDX{m_translationKP, m_translationKI, m_translationKD, m_translationConstraints, 20_ms};
    frc::ProfiledPIDController<units::length::meters> m_translationPIDY{m_translationKP, m_translationKI, m_translationKD, m_translationConstraints, 20_ms};
};
