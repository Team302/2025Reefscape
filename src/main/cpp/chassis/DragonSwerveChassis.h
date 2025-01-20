
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
#include <map>
#include <string>

#include "frc/estimator/SwerveDrivePoseEstimator.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/mass.h"
#include "units/moment_of_inertia.h"
#include "units/velocity.h"
#include "wpi/DataLog.h"
// #include "grpl/LaserCan.h"

#include "chassis/ChassisOptionEnums.h"
#include "chassis/states/ISwerveDriveState.h"
#include "chassis/states/ISwerveDriveOrientation.h"
#include "chassis/IChassis.h"
#include "chassis/DragonSwerveModule.h"
#include "chassis/ChassisMovement.h"
#include "vision/DragonVision.h"
#include "utils/logging/DragonDataLogger.h"
#include "utils/logging/LoggableItem.h"

#include "ctre/phoenix6/Pigeon2.hpp"
#include "ctre/phoenix6/swerve/SwerveDrivetrain.hpp"

class RobotDrive;

class DragonSwerveChassis : public IChassis, public LoggableItem, public DragonDataLogger
{
public:
    /// @brief Construct a swerve chassis
    DragonSwerveChassis(DragonSwerveModule *frontLeft,
                        DragonSwerveModule *frontRight,
                        DragonSwerveModule *backLeft,
                        DragonSwerveModule *backRight,
                        ctre::phoenix6::hardware::Pigeon2 *pigeon,
                        std::string configfilename,
                        std::string networkTableName);
    DragonSwerveChassis();
    ~DragonSwerveChassis() noexcept override = default;

    enum SWERVE_MODULES
    {
        LEFT_FRONT,
        RIGHT_FRONT,
        LEFT_BACK,
        RIGHT_BACK,
    };

    void InitStates();

    /// @brief Align all of the swerve modules to point forward
    void ZeroAlignDragonSwerveModules();

    /// @brief Drive the chassis
    void Drive(ChassisMovement &moveInfo) override;

    /// @brief update the chassis odometry based on current states of the swerve modules and the pigeon
    void UpdateOdometry();

    /// @brief Provide the current chassis speed information
    frc::ChassisSpeeds GetChassisSpeeds() const;

    /// @brief Get encoder values
    double GetEncoderValues(DragonSwerveModule *motor);

    /// @brief Reset the current chassis pose based on the provided pose (the rotation comes from the Pigeon)
    /// @param [in] const Pose2d&       pose        Current XY position
    void ResetPose(const frc::Pose2d &pose) override;

    /// @brief Reset yaw to 0 or 180 degrees depending on alliance
    void ResetYaw();
    void SetYaw(units::angle::degree_t newYaw);

    void LogSwerveEncoderData(DragonSwerveChassis::SWERVE_MODULES DragonSwerveModule);

    units::length::inch_t GetWheelDiameter() const override;
    units::length::inch_t GetWheelBase() const { return m_wheelBase; }
    units::length::inch_t GetTrack() const { return m_track; }
    units::velocity::meters_per_second_t GetMaxSpeed() const override;
    units::angular_velocity::radians_per_second_t GetMaxAngularSpeed() const override;

    DragonSwerveModule *GetFrontLeft() const { return m_frontLeft; }
    DragonSwerveModule *GetFrontRight() const { return m_frontRight; }
    DragonSwerveModule *GetBackLeft() const { return m_backLeft; }
    DragonSwerveModule *GetBackRight() const { return m_backRight; }
    frc::Pose2d GetPose() const;
    units::angle::degree_t GetYaw() const override;
    units::angle::degree_t GetPitch() const;
    units::angle::degree_t GetRoll() const;

    frc::SwerveDriveKinematics<4> GetKinematics() const { return m_kinematics; }

    // Dummy functions for IChassis Implementation
    inline IChassis::CHASSIS_TYPE GetType() const override { return IChassis::CHASSIS_TYPE::SWERVE; };
    inline void Initialize() override {};

    void SetTargetHeading(units::angle::degree_t targetYaw) override;

    void SetStoredHeading(units::angle::degree_t heading);
    units::angle::degree_t GetStoredHeading() const { return m_storedYaw; };

    ISwerveDriveOrientation *GetSpecifiedHeadingState(ChassisOptionEnums::HeadingOption headingOption);
    ISwerveDriveState *GetSpecifiedDriveState(ChassisOptionEnums::DriveStateType driveOption);

    bool IsRotating() const { return m_isRotating; }
    double GetRotationRateDegreesPerSecond() const { return m_pigeon != nullptr ? m_pigeon->GetAngularVelocityZWorld(true).GetValueAsDouble() : 0.0; }
    units::velocity::meters_per_second_t GetInertialVelocity();
    void LogInformation() override;
    void DataLog() override;

    units::mass::kilogram_t GetMass() const { return m_mass; }
    units::moment_of_inertia::kilogram_square_meter_t GetMomenOfInertia() const { return m_momentOfInertia; }

    // std::optional<uint16_t> GetLaserValue();

private:
    // ctre::phoenix6::swerve::SwerveDrivetrain *m_swervedrive;

    ISwerveDriveOrientation *GetHeadingState(const ChassisMovement &moveInfo);
    ISwerveDriveState *GetDriveState(ChassisMovement &moveInfo);
    void ReadConstants(std::string configfilename);

    DragonSwerveModule *m_frontLeft;
    DragonSwerveModule *m_frontRight;
    DragonSwerveModule *m_backLeft;
    DragonSwerveModule *m_backRight;
    ctre::phoenix6::hardware::Pigeon2 *m_pigeon;

    RobotDrive *m_robotDrive;
    std::map<ChassisOptionEnums::DriveStateType, ISwerveDriveState *> m_driveStateMap;
    std::map<ChassisOptionEnums::HeadingOption, ISwerveDriveOrientation *> m_headingStateMap;

    units::length::inch_t m_wheelBase = units::length::inch_t(22.75);
    units::length::inch_t m_track = units::length::inch_t(22.75);
    units::velocity::feet_per_second_t m_maxSpeed = units::velocity::feet_per_second_t(17.3);
    units::length::inch_t m_wheelDiameter = units::length::inch_t(4.0);
    units::length::meter_t m_radius;

    units::velocity::meters_per_second_t m_drive = units::velocity::meters_per_second_t(0.0);
    units::velocity::meters_per_second_t m_steer = units::velocity::meters_per_second_t(0.0);
    units::angular_velocity::radians_per_second_t m_rotate = units::angular_velocity::radians_per_second_t(0.0);

    static constexpr units::velocity::meters_per_second_t m_velocityDeadband = units::velocity::meters_per_second_t(0.025);
    static constexpr units::angular_velocity::radians_per_second_t m_angularDeadband = units::angular_velocity::radians_per_second_t(0.1);

    frc::Translation2d m_frontLeftLocation;
    frc::Translation2d m_frontRightLocation;
    frc::Translation2d m_backLeftLocation;
    frc::Translation2d m_backRightLocation;

    frc::SwerveDriveKinematics<4> m_kinematics;

    frc::SwerveDrivePoseEstimator<4> m_poseEstimator;

    units::angle::degree_t m_storedYaw;

    units::angle::degree_t m_targetHeading;

    ISwerveDriveState *m_currentDriveState;
    ISwerveDriveOrientation *m_currentOrientationState;

    bool m_initialized = false;
    std::string m_networkTableName;
    bool m_isRotating = false;
    bool m_rotatingLatch = false;
    bool m_initDataLog = false;
    DragonVision *m_vision;
    std::array<frc::SwerveModuleState, 4U> m_targetStates;

    units::mass::kilogram_t m_mass = units::mass::kilogram_t(52.0);                                                                // TODO put a real value in
    units::moment_of_inertia::kilogram_square_meter_t m_momentOfInertia = units::moment_of_inertia::kilogram_square_meter_t(26.0); // TODO put a real value in

    frc::Timer m_velocityTimer;

    // grpl::LaserCan *m_laserCan = nullptr;
    // void DefineLaserCan(grpl::LaserCanRangingMode rangingMode, grpl::LaserCanROI roi, grpl::LaserCanTimingBudget timingBudget);
};