
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

#include <frc/geometry/Pose2d.h>

#include "auton/AutonGrid.h"
#include "auton/ZoneParams.h"
#include "chassis/ChassisOptionEnums.h"

// #include "mechanisms/noteManager/generated/noteManagerGen.h"

// @ADDMECH include for your mechanism state mgr

// @ADDMECH mechanism state for mech as parameter
ZoneParams::ZoneParams(AutonGrid::XGRID xgrid1,
					   AutonGrid::YGRID ygrid1,
					   AutonGrid::XGRID xgrid2,
					   AutonGrid::YGRID ygrid2,
					   frc::Pose2d circlePose,
					   units::inch_t radius,
					   bool isTaleStateChanging,
					   bool isIntakeStateChanging,
					   IntakeManager::STATE_NAMES intakeOption,
					   DragonTale::STATE_NAMES taleOption,
					   ChassisOptionEnums::AutonChassisOptions autonchassisoption,
					   ChassisOptionEnums::HeadingOption headingOption,
					   ChassisOptionEnums::AutonAvoidOptions autonavoidoption, AutonGrid::ZoneMode zoneMode) : m_xgrid1(xgrid1),
																											   m_ygrid1(ygrid1),
																											   m_xgrid2(xgrid2),
																											   m_ygrid2(ygrid2),
																											   m_isIntakeStateChanging(isIntakeStateChanging),
																											   m_isTaleStateChanging(isTaleStateChanging),
																											   m_intakeOption(intakeOption),
																											   m_taleOption(taleOption),
																											   m_chassisoption(autonchassisoption),
																											   m_headingOption(headingOption),
																											   m_avoidoption(autonavoidoption),
																											   m_zoneMode(zoneMode),
																											   m_circlePose(circlePose),
																											   m_radius(radius)
{
}
