
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

// #include <array>
#include <map>

#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Pose3d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "wpi/DataLog.h"

class DragonDataLoggerSignals
{
    friend class DragonDataLogger;

public:
    // when adding a signal:
    // - Add to the appropriate enum
    // - Add a signal variable below
    // - Add a current value for the signal below
    // - Construct the signal in DragonDataLoggerSignals::DragonDataLoggerSignals()
    // - Add Case statement in the appropriate helper method in DragonDataLogger

    enum BoolSignals
    {
        FRONT_LEFT_SWERVE_MODULE_SPLIPING,
        FRONT_RIGHT_SWERVE_MODULE_SPLIPING,
        BACK_LEFT_SWERVE_MODULE_SPLIPING,
        BACK_RIGHT_SWERVE_MODULE_SPLIPING
    };

    enum DoubleSignals
    {
        CHASSIS_STORED_HEADING_DEGREES,
        CHASSIS_YAW_DEGREES,
        ELECTRICAL_VOLTAGE,
        ELECTRICAL_CURRENT,
        ELECTRICAL_ENERGY,
        ELECTRICAL_POWER,
        LIMELIGHT_TV_1,
        LIMELIGHT_TX_1,
        LIMELIGHT_TY_1,
        LIMELIGHT_FIDUCIAL_ID_1
    };

    enum StringSignals
    {
        CHASSIS_HEADING_STATE,
        CHASSIS_DRIVE_STATE,
        AUTON_PATH_NAME
    };

    enum PoseSingals
    {
        CURRENT_CHASSIS_POSE2D,
        CURRENT_CHASSIS_LIMELIGHT_POSE3D,
        CURRENT_CHASSIS_LIMELIGHT2_POSE3D,
        CURRENT_CHASSIS_QUEST_POSE3D
    };

    enum ChassisSpeedSignals
    {
        TARGET_SPEEDS,
        ACTUAL_SPEEDS
    };

    enum SwerveStateSingals
    {
        TARGET_LEFT_FRONT_STATE,
        TARGET_RIGHT_FRONT_STATE,
        TARGET_LEFT_BACK_STATE,
        TARGET_RIGHT_BACK_STATE,
        ACTUAL_LEFT_FRONT_STATE,
        ACTUAL_RIGHT_FRONT_STATE,
        ACTUAL_LEFT_BACK_STATE,
        ACTUAL_RIGHT_BACK_STATE
    };

    static DragonDataLoggerSignals *GetInstance();

private:
    // initialize these signals in the constructor
    wpi::log::BooleanLogEntry m_hasVision;
    bool m_currHasVision = false;

    wpi::log::BooleanLogEntry m_frontSensor;
    bool m_currFrontSensor = false;

    wpi::log::BooleanLogEntry m_backSensor;
    bool m_currBackSensor = false;

    wpi::log::BooleanLogEntry m_launcherSensor;
    bool m_currLauncherSensor = false;

    wpi::log::BooleanLogEntry m_feederSensor;
    bool m_currFeederSensor = false;

    wpi::log::BooleanLogEntry m_placerInSensor;
    bool m_currPlacerInSensor = false;

    wpi::log::BooleanLogEntry m_placerMidSensor;
    bool m_currPlacerMidSensor = false;

    wpi::log::BooleanLogEntry m_placerOutSensor;
    bool m_currPlacerOutSensor = false;

    wpi::log::BooleanLogEntry m_frontLeftSwerveModuleslip;
    bool m_currFrontLeftSwerveModuleslip = false;

    wpi::log::BooleanLogEntry m_frontRightSwerveModuleslip;
    bool m_currFrontRightSwerveModuleslip = false;

    wpi::log::BooleanLogEntry m_backLeftSwerveModuleslip;
    bool m_currBackLeftSwerveModuleslip = false;

    wpi::log::BooleanLogEntry m_backRightSwerveModuleslip;
    bool m_currBackRightSwerveModuleslip = false;

    wpi::log::DoubleLogEntry m_storedHeading;
    double m_currStoredHeading{0.0};

    wpi::log::DoubleLogEntry m_chassisYaw;
    double m_currChassisYaw{0.0};

    wpi::log::DoubleLogEntry m_nmTargetAngle;
    double m_currNmTargetAngle{0.0};

    wpi::log::DoubleLogEntry m_nmActualAngle;
    double m_currNmActualAngle{0.0};

    wpi::log::DoubleLogEntry m_nmTopTarget;
    double m_currNmTopTarget{0.0};

    wpi::log::DoubleLogEntry m_nmBottomTarget;
    double m_currNmBottomTarget{0.0};

    wpi::log::DoubleLogEntry m_nmTopActual;
    double m_currNmTopActual{0.0};

    wpi::log::DoubleLogEntry m_nmBottomActual;
    double m_currNmBottomActual{0.0};

    wpi::log::DoubleLogEntry m_distFromSpeaker;
    double m_currDistFromSpeaker{0.0};

    wpi::log::DoubleLogEntry m_electricalVoltage;
    double m_currElectricalVoltage{0.0};

    wpi::log::DoubleLogEntry m_electricalCurrent;
    double m_currElectricalCurrent{0.0};

    wpi::log::DoubleLogEntry m_electricalEnergy;
    double m_currElectricalEnergy{0.0};

    wpi::log::DoubleLogEntry m_electricalPower;
    double m_currElectricalPower{0.0};

    wpi::log::DoubleLogEntry m_tx;
    double m_currTx{0.0};

    wpi::log::DoubleLogEntry m_ty;
    double m_currTy{0.0};

    wpi::log::DoubleLogEntry m_tv;
    double m_currTv{0.0};

    wpi::log::DoubleLogEntry m_Fiducial;
    double m_currFiducial{0.0};

    wpi::log::StringLogEntry m_headingState;
    std::string m_currHeadingState{""};

    wpi::log::StringLogEntry m_driveState;
    std::string m_currDriveState{""};

    wpi::log::StructLogEntry<frc::Pose2d> m_pose2d;
    wpi::log::StructLogEntry<frc::Pose3d> m_pose3dLimelight;
    wpi::log::StructLogEntry<frc::Pose3d> m_pose3dLimelight2;
    wpi::log::StructLogEntry<frc::Pose3d> m_pose3dQuest;

    frc::Pose2d m_currPose2D{};
    frc::Pose3d m_currPose3DLimelight{};
    frc::Pose3d m_currPose3DLimelight2{};
    frc::Pose3d m_currPose3DQuest{};

    wpi::log::StructLogEntry<frc::SwerveModuleState> m_frontLeftTarget;
    frc::SwerveModuleState m_currFrontLeftTarget{};

    wpi::log::StructLogEntry<frc::SwerveModuleState> m_frontRightTarget;
    frc::SwerveModuleState m_currFrontRightTarget{};

    wpi::log::StructLogEntry<frc::SwerveModuleState> m_backLeftTarget;
    frc::SwerveModuleState m_currBackLeftTarget{};

    wpi::log::StructLogEntry<frc::SwerveModuleState> m_backRightTarget;
    frc::SwerveModuleState m_currBackRightTarget{};

    wpi::log::StructLogEntry<frc::SwerveModuleState> m_frontLeftActual;
    frc::SwerveModuleState m_currFrontLeftActual{};

    wpi::log::StructLogEntry<frc::SwerveModuleState> m_frontRightActual;
    frc::SwerveModuleState m_currFrontRightActual{};

    wpi::log::StructLogEntry<frc::SwerveModuleState> m_backLeftActual;
    frc::SwerveModuleState m_currBackLeftActual{};

    wpi::log::StructLogEntry<frc::SwerveModuleState> m_backRightActual;
    frc::SwerveModuleState m_currBackRightActual{};

    wpi::log::StructLogEntry<frc::ChassisSpeeds> m_targetSpeeds;
    frc::ChassisSpeeds m_currTargetSpeeds{};

    wpi::log::StructLogEntry<frc::ChassisSpeeds> m_actualSpeeds;
    frc::ChassisSpeeds m_currActualSpeeds{};

    DragonDataLoggerSignals();
    virtual ~DragonDataLoggerSignals() = delete;

    static DragonDataLoggerSignals *m_instance;
};
