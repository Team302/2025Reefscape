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

// C++ Includes
#include <memory>
#include <string>

// FRC Includes
#include "frc/filter/SlewRateLimiter.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "frc/system/plant/DCMotor.h"
#include "networktables/DoubleTopic.h"
#include "networktables/NetworkTable.h"
#include "units/angular_velocity.h"
#include "units/dimensionless.h"
#include "units/velocity.h"
#include "utils/logging/debug/LoggableItem.h"

// Team 302 Includes
#include "chassis/SwerveModuleConstants.h"

// Third Party
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/CANcoder.hpp"
#include "pathplanner/lib/config/ModuleConfig.h"

class SwerveModule : public LoggableItem
{
public:
    SwerveModule() = delete;
    ~SwerveModule() = default;

    /// @brief Constructs a Swerve Module.  This is assuming 2 TalonFX (Falcons) with a CanCoder for the turn angle
    SwerveModule(std::string canbusname,
                 SwerveModuleConstants::ModuleID id,
                 units::length::inch_t wheelDiameter,
                 units::dimensionless::scalar_t driveGearRatio,
                 double sensorToMechanismRatio,
                 units::dimensionless::scalar_t rotorToSensorRatio,
                 units::velocity::feet_per_second_t maxSpeed,
                 int driveMotorID,
                 bool driveInverted,
                 int turnMotorID,
                 bool turnInverted,
                 int canCoderID,
                 bool canCoderInverted,
                 const units::angle::turn_t angleOffset,
                 ctre::phoenix6::configs::Slot0Configs turnGains,
                 // std::string configfilename,
                 std::string networkTableName);

    /// @brief Turn all of the wheel to zero degrees yaw according to the pigeon
    /// @returns void
    void ZeroAlignModule();

    ///@brief
    /// @returns
    double GetEncoderValues();

    /// @brief Get the current state of the module (speed of the wheel and angle of the wheel)
    /// @returns SwerveModuleState
    frc::SwerveModuleState GetState() const;
    frc::SwerveModuleState GetOptimizedState() const { return m_optimizedState; }
    frc::SwerveModulePosition GetPosition() const;

    /// @brief Set the current state of the module (speed of the wheel and angle of the wheel)
    /// @param [in] SwerveModuleState& referenceState:   state to set the module to
    /// @returns void
    void SetDesiredState(frc::SwerveModuleState &state);

    void RunCurrentState();

    /// @brief Calculate the real speed of the module incorporating the inertial velocity and rotate rate
    /// * @param inertialVelocity Inertial velocity of robot (m/s)
    /// * @param rotateRate Rotate rate of robot(degrees / s)
    /// * @param radius Radius of swerve drive (center to the furtherest wheel)
    /// @returns velcity::meters_per_second_t
    units::velocity::meters_per_second_t CalculateRealSpeed(units::velocity::meters_per_second_t inertialVelocity, units::angular_velocity::radians_per_second_t rotateRate, units::length::meter_t radius);

    /// @brief Return which module this is
    /// @returns SwerveModuleConstants.ModuleID
    SwerveModuleConstants::ModuleID GetModuleID() { return m_moduleID; }

    frc::DCMotor GetDriveMotorDef() const { return m_driveMotorDef; }
    frc::DCMotor GetSteerMotorDef() const { return m_steerMotorDef; }
    units::length::inch_t GetWheelDiameter() const { return m_wheelDiameter; }
    units::velocity::feet_per_second_t GetMaxSpeed() const { return m_maxSpeed; }
    double GetCoefficientOfFriction() const { return m_coeffOfFriction; }
    units::current::ampere_t GetDriveCurrentLimit() const { return m_driveTalon->GetStatorCurrent().GetValue(); }
    units::current::ampere_t GetSteerCurrentLimit() const { return m_steerTalon->GetStatorCurrent().GetValue(); }

    void StopMotors();
    void LogInformation() override;

    pathplanner::ModuleConfig GetModuleConfig() { return m_moduleConfig; }

    // std::optional<uint16_t> GetLaserValue();

private:
    void InitDriveMotor(bool inverted);
    void InitSteerMotorEncoder(
        bool turnInverted,
        bool canCoderInverted,
        const units::angle::turn_t angleOffset,
        double sensorToMechanismRatio,
        units::dimensionless::scalar_t rotorToSensorRatio);
    void SetDriveSpeed(units::velocity::meters_per_second_t speed);
    void SetSteerAngle(units::angle::degree_t angle);
    frc::DCMotor m_driveMotorDef = frc::DCMotor::KrakenX60FOC();
    frc::DCMotor m_steerMotorDef = frc::DCMotor::KrakenX60FOC();

    SwerveModuleConstants::ModuleID m_moduleID;
    ctre::phoenix6::hardware::TalonFX *m_driveTalon;
    ctre::phoenix6::hardware::TalonFX *m_steerTalon;
    ctre::phoenix6::hardware::CANcoder *m_steerCancoder;

    frc::SwerveModuleState m_activeState;
    frc::SwerveModuleState m_optimizedState;

    ctre::phoenix6::controls::PositionTorqueCurrentFOC m_positionTorque = ctre::phoenix6::controls::PositionTorqueCurrentFOC{0_tr}.WithSlot(0);
    ctre::phoenix6::controls::PositionVoltage m_positionVoltage = ctre::phoenix6::controls::PositionVoltage{0_tr}.WithSlot(0);
    ctre::phoenix6::controls::VelocityTorqueCurrentFOC m_velocityTorque = ctre::phoenix6::controls::VelocityTorqueCurrentFOC{0_tps}.WithSlot(0);
    ctre::phoenix6::controls::VelocityVoltage m_velocityVoltage = ctre::phoenix6::controls::VelocityVoltage{0_tps}.WithSlot(0);

    // Drive Motor Gains
    double m_driveKp = 0.0;
    double m_driveKi = 0.0;
    double m_driveKd = 0.0;
    double m_driveKs = 0.0;
    double m_driveKf = 0.0;

    double m_gearRatio = 0.0;
    double m_steerCruiseVel = 0.0;
    double m_steerMaxAcc = 0.0;
    units::length::inch_t m_wheelDiameter = units::length::inch_t(4.0);
    units::velocity::feet_per_second_t m_maxSpeed = units::velocity::feet_per_second_t(17.3);
    double m_coeffOfFriction = 0.9;

    ctre::phoenix6::configs::Slot0Configs m_steerGains;

    bool m_velocityControlled = false;
    bool m_useFOC = false;
    std::string m_networkTableName;
    pathplanner::ModuleConfig m_moduleConfig;
};