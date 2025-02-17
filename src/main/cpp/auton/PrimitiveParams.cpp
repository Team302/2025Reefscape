
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

#include "auton/PrimitiveEnums.h"
#include "auton/PrimitiveParams.h"
#include "chassis/ChassisOptionEnums.h"

// @ADDMECH mechanism state for mech as parameter
PrimitiveParams::PrimitiveParams(PRIMITIVE_IDENTIFIER id,
								 units::time::second_t time,
								 ChassisOptionEnums::HeadingOption headingOpt,
								 float heading,
								 std::string pathName,
								 std::string choreoTrajectoryName,
								 ChassisOptionEnums::PathGainsType pathgainstype,
								 ZoneParamsVector zones,
								 VISION_ALIGNMENT visionAlignment,
								 bool intakeStateChanged,
								 IntakeManager::STATE_NAMES intakeState,
								 bool taleStateChanged,
								 DragonTale::STATE_NAMES taleState,
								 ChassisOptionEnums::PathUpdateOption updatePathOption,
								 PATH_UPDATE_OPTION updateOption,
								 DriveStopDelay::DelayOption delayOption) : m_id(id), // Primitive ID
																			m_time(time),
																			m_headingOption(headingOpt),
																			m_heading(heading),
																			m_delayOption(delayOption),
																			m_pathName(pathName),
																			m_choreoTrajectoryName(choreoTrajectoryName),
																			m_pathGainsType(pathgainstype),
																			m_visionAlignment(visionAlignment),
																			m_changeIntakeState(intakeStateChanged),
																			m_changeTaleState(taleStateChanged),
																			m_intakeState(intakeState),
																			m_taleState(taleState),
																			m_zones(zones),
																			m_pathUpdateOption(updatePathOption),
																			m_updateOption(updateOption)

// @ADDMECH initilize state mgr attribute
{
}
