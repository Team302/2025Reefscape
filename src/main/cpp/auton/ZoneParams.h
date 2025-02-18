
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
#include <vector>

// FRC includes

// Team 302 includes
#include "auton/AutonGrid.h"
#include "chassis/ChassisOptionEnums.h"
#include "mechanisms/IntakeManager/IntakeManager.h"
#include "mechanisms/DragonTale/DragonTale.h"
#include "auton/PrimitiveEnums.h"
// Third Party Includes

class ZoneParams
{
public:
    ZoneParams(AutonGrid::XGRID xgrid1,
               AutonGrid::YGRID ygrid1,
               AutonGrid::XGRID xgrid2,
               AutonGrid::YGRID ygrid2,
               frc::Pose2d circlePose,
               units::length::inch_t radius,
               bool isTaleStateChanging,
               bool isIntakeStateChanging,
               IntakeManager::STATE_NAMES intakeOption,
               DragonTale::STATE_NAMES taleOption,
               ChassisOptionEnums::AutonChassisOptions autonchassisoption,
               ChassisOptionEnums::HeadingOption headingOption,
               PATH_UPDATE_OPTION updateOption,
               ChassisOptionEnums::AutonAvoidOptions autonavoidoption,
               AutonGrid::ZoneMode zoneMode); // declare ZoneParams public constructor with parameters xgrid1, etc.

    ZoneParams() = delete;
    ~ZoneParams() = default; // Destructor

    AutonGrid::XGRID GetXGrid1() const { return m_xgrid1; }
    AutonGrid::XGRID GetXGrid2() const { return m_xgrid2; }
    AutonGrid::YGRID GetYGrid1() const { return m_ygrid1; }
    AutonGrid::YGRID GetYGrid2() const { return m_ygrid2; }

    AutonGrid::ZoneMode GetZoneMode() const { return m_zoneMode; }

    frc::Pose2d getCircleZonePose() const { return m_circlePose; }
    units::length::inch_t getRadius() const { return m_radius; }

    bool IsIntakeStateChanging() const { return m_isIntakeStateChanging; }
    bool IsTaleStateChanging() const { return m_isTaleStateChanging; }

    IntakeManager::STATE_NAMES GetIntakeOption() const { return m_intakeOption; }
    DragonTale::STATE_NAMES GetTaleOption() const { return m_taleOption; }

    ChassisOptionEnums::AutonChassisOptions GetChassisOption() const { return m_chassisoption; }
    ChassisOptionEnums::AutonAvoidOptions GetAvoidOption() const { return m_avoidoption; }

private:
    AutonGrid::XGRID m_xgrid1;
    AutonGrid::YGRID m_ygrid1;
    AutonGrid::XGRID m_xgrid2;
    AutonGrid::YGRID m_ygrid2;
    bool m_isIntakeStateChanging;
    bool m_isTaleStateChanging;

    IntakeManager::STATE_NAMES m_intakeOption;
    DragonTale::STATE_NAMES m_taleOption;

    ChassisOptionEnums::AutonChassisOptions m_chassisoption;
    ChassisOptionEnums::HeadingOption m_headingOption;
    ChassisOptionEnums::AutonAvoidOptions m_avoidoption; // instances of said parameters

    PATH_UPDATE_OPTION m_updateOption;

    AutonGrid::ZoneMode m_zoneMode;

    frc::Pose2d m_circlePose;
    units::length::inch_t m_radius;
};

typedef std::vector<ZoneParams *> ZoneParamsVector; // create typedef ZoneParamsVector
