
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

#include <frc/TimedRobot.h>

#include "frc/geometry/Pose2d.h"

class CyclePrimitives;
class HolonomicDrive;
class SwerveChassis;
class TeleopControl;
class FMSData;
class DragonField;
class DragonPower;
class AutonPreviewer;
class RobotState;
class SomeMech;
class DragonDataLoggerMgr;
class DragonSwervePoseEstimator;

class Robot : public frc::TimedRobot
{
public:
    Robot() = default;
    ~Robot() = default;

    void RobotInit() override;
    void RobotPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void TestInit() override;
    void TestPeriodic() override;
    void SimulationInit() override;
    void SimulationPeriodic() override;

private:
    void InitializeRobot();
    void InitializeAutonOptions();
    void InitializeDriveteamFeedback();
    void InitializeDataTracing();
    void UpdateDriveTeamFeedback();

    TeleopControl *m_controller;
    SwerveChassis *m_chassis;
    CyclePrimitives *m_cyclePrims;
    HolonomicDrive *m_holonomic;
    DragonPower *m_dragonPower;

    FMSData *m_fmsData;
    DragonField *m_field;
    AutonPreviewer *m_previewer;
    RobotState *m_robotState;
    SomeMech *m_someMech;
    DragonDataLoggerMgr *m_datalogger;
    bool isFMSAttached = false;
    DragonSwervePoseEstimator *m_dragonswerveposeestimator;
};
