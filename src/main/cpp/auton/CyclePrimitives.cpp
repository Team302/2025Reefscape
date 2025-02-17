
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

// C++ Includes
#include <memory>
#include <string>

// FRC includes
#include "frc/DriverStation.h"
#include "frc/Timer.h"

// Team 302 includes
#include "auton/AutonGrid.h"
#include "auton/AutonSelector.h"
#include "auton/CyclePrimitives.h"
#include "auton/PrimitiveEnums.h"
#include "auton/PrimitiveFactory.h"
#include "auton/PrimitiveParams.h"
#include "auton/PrimitiveParser.h"
#include "auton/drivePrimitives/IPrimitive.h"
#include "utils/logging/debug/Logger.h"
#include "chassis/definitions/ChassisConfig.h"
#include "chassis/definitions/ChassisConfigMgr.h"
#include "chassis/ChassisOptionEnums.h"
#include "chassis/SwerveModule.h"
#include "mechanisms/DragonTale/DragonTale.h"
#include "mechanisms/IntakeManager/IntakeManager.h"
// #include "mechanisms/MechanismTypes.h"

// Third Party Includes

using frc::DriverStation;
using frc::Timer;
using std::make_unique;
using std::string;
#include <pugixml/pugixml.hpp>
using namespace pugi;

CyclePrimitives::CyclePrimitives() : State(string("CyclePrimitives"), 0),
									 m_primParams(),
									 m_currentPrimSlot(0),
									 m_currentPrim(nullptr),
									 m_primFactory(PrimitiveFactory::GetInstance()),
									 m_driveStop(nullptr),
									 m_autonSelector(new AutonSelector()),
									 m_timer(make_unique<Timer>()),
									 m_maxTime(units::time::second_t(0.0)),
									 m_isDone(false),
									 m_chassis(),
									 m_updatedHeadingOption()
{
	auto chassisConfig = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
	m_chassis = chassisConfig != nullptr ? chassisConfig->GetSwerveChassis() : nullptr;
}

void CyclePrimitives::Init()
{
	m_primParams.clear();
	m_currentPrimSlot = 0; // Reset current prim
	m_currentPrim = nullptr;
	m_zones.clear();

	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("CyclePrim"), string("About to parse XML file "), m_autonSelector->GetSelectedAutoFile().c_str());

	m_primParams = PrimitiveParser::ParseXML(m_autonSelector->GetSelectedAutoFile());
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("CyclePrim"), string("nPrims"), double(m_primParams.size()));

	InitDriveStopDelayTimes();
	if (!m_primParams.empty())
	{
		GetNextPrim();
	}

	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("CyclePrim"), string("end init"), m_autonSelector->GetSelectedAutoFile().c_str());
}

void CyclePrimitives::Run()
{
	Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("CyclePrim"), string("Arrived at "), string("run"));
	if (m_currentPrim != nullptr)
	{
		Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("CyclePrim"), string("CurrentPrim "), string("run"));
		m_currentPrim->Run();

		if (m_chassis != nullptr)
		{

			Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("CyclePrim"), string("CurrentPrimSlot "), m_currentPrimSlot);
			Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("CyclePrim"), string("Prim Size "), (int)m_primParams.size());
			Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("CyclePrim"), string("zone size "), (int)m_zones.size());

			if (!m_zones.empty())
			{

				for (auto zone : m_zones)
				{
					bool isInZone = false;
					if (zone != nullptr)
					{
						if (zone->GetZoneMode() == AutonGrid::RECTANGLE)
						{
							isInZone = AutonGrid::GetInstance()->IsPoseInZone(zone->GetXGrid1(),
																			  zone->GetXGrid2(),
																			  zone->GetYGrid1(),
																			  zone->GetYGrid2(),
																			  m_chassis->GetPose());
						}
						else if (zone->GetZoneMode() == AutonGrid::CIRCLE)
						{
							isInZone = AutonGrid::GetInstance()->IsPoseInZone(zone->getCircleZonePose(),
																			  zone->getRadius(),
																			  m_chassis->GetPose());
						}

						if (isInZone)
						{

							SetMechanismStatesFromZone(zone);

							if (zone->GetChassisOption() != ChassisOptionEnums::AutonChassisOptions::NO_VISION)
							{
								// TODO:  plug in vision drive options
							}

							if (zone->GetAvoidOption() != ChassisOptionEnums::AutonAvoidOptions::NO_AVOID_OPTION)
							{
								// TODO:  plug in avoid options
							}
						}
					}
				}
			}
		}

		if (m_currentPrim->IsDone())
		{
			GetNextPrim();
		}
	}
	else
	{
		Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("CyclePrim"), string("CurrentPrim "), string("done"));
		m_isDone = true;
		m_primParams.clear();  // clear the primitive params vector
		m_currentPrimSlot = 0; // Reset current prim slot
		RunDriveStop();
	}
}

void CyclePrimitives::Exit()
{
}

bool CyclePrimitives::AtTarget()
{
	return m_isDone;
}

void CyclePrimitives::GetNextPrim()
{
	if (!m_primParams.empty())
	{
		PrimitiveParams *currentPrimParam = (m_currentPrimSlot < (int)m_primParams.size()) ? m_primParams[m_currentPrimSlot] : nullptr;

		m_currentPrim = (currentPrimParam != nullptr) ? m_primFactory->GetIPrimitive(currentPrimParam) : nullptr;
		if (m_currentPrim != nullptr)
		{
			m_currentPrim->Init(currentPrimParam);

			SetMechanismStatesFromParam(currentPrimParam);
			m_zones = currentPrimParam->GetZones();

			m_maxTime = currentPrimParam->GetTime();
			m_timer->Reset();
			m_timer->Start();
		}

		Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("CyclePrim"), string("Current Prim "), m_currentPrimSlot);

		m_currentPrimSlot++;
	}
}

void CyclePrimitives::RunDriveStop()
{
	if (m_driveStop == nullptr)
	{
		auto time = DriverStation::GetMatchType() != DriverStation::MatchType::kNone ? DriverStation::GetMatchTime() : units::time::second_t(15.0);
		auto params = new PrimitiveParams(DO_NOTHING, // identifier
										  time,		  // time
										  ChassisOptionEnums::HeadingOption::MAINTAIN,
										  0.0,		// heading
										  string(), // pathname
										  string(), // ChoreoTrajectoryName
										  ChassisOptionEnums::PathGainsType::LONG,
										  ZoneParamsVector(),
										  PrimitiveParams::VISION_ALIGNMENT::UNKNOWN,
										  false,
										  IntakeManager::STATE_NAMES::STATE_OFF,
										  false,
										  DragonTale::STATE_NAMES::STATE_READY,
										  ChassisOptionEnums::PathUpdateOption::NONE,
										  PATH_UPDATE_OPTION::NOTHING,
										  DriveStopDelay::DelayOption::START);
		m_driveStop = m_primFactory->GetIPrimitive(params);
		m_driveStop->Init(params);
	}
	m_driveStop->Run();
}

void CyclePrimitives::SetMechanismStatesFromParam(PrimitiveParams *params)
{

	auto config = MechanismConfigMgr::GetInstance()->GetCurrentConfig();
	if (params != nullptr && config != nullptr)
	{
		auto intakeStateMgr = config->GetMechanism(MechanismTypes::MECHANISM_TYPE::INTAKE_MANAGER);
		auto intakeMgr = intakeStateMgr != nullptr ? dynamic_cast<IntakeManager *>(intakeStateMgr) : nullptr;

		if (intakeMgr != nullptr && params->IsIntakeStateChanging())
		{
			intakeMgr->SetCurrentState(params->GetIntakeState(), true);
		}

		auto taleStateMgr = config->GetMechanism(MechanismTypes::MECHANISM_TYPE::DRAGON_TALE);
		auto taleMgr = taleStateMgr != nullptr ? dynamic_cast<DragonTale *>(taleStateMgr) : nullptr;

		if (taleMgr != nullptr && params->IsTaleStateChanging())
		{
			taleMgr->SetCurrentState(params->GetTaleState(), true);
		}
	}
}
void CyclePrimitives::SetMechanismStatesFromZone(ZoneParams *params)
{
	auto config = MechanismConfigMgr::GetInstance()->GetCurrentConfig();
	if (params != nullptr && config != nullptr)
	{
		auto intakeStateMgr = config->GetMechanism(MechanismTypes::MECHANISM_TYPE::INTAKE_MANAGER);
		auto intakeMgr = intakeStateMgr != nullptr ? dynamic_cast<IntakeManager *>(intakeStateMgr) : nullptr;

		if (intakeMgr != nullptr && params->IsIntakeStateChanging())
		{
			intakeMgr->SetCurrentState(params->GetIntakeOption(), true);
		}

		auto taleStateMgr = config->GetMechanism(MechanismTypes::MECHANISM_TYPE::DRAGON_TALE);
		auto taleMgr = taleStateMgr != nullptr ? dynamic_cast<DragonTale *>(taleStateMgr) : nullptr;

		if (taleMgr != nullptr && params->IsTaleStateChanging())
		{
			taleMgr->SetCurrentState(params->GetTaleOption(), true);
		}
	}
}
void CyclePrimitives::InitDriveStopDelayTimes()
{
	if (!m_primParams.empty())
	{
		auto startDelay = m_autonSelector->GetStartDelay();
		auto reefDelay = m_autonSelector->GetReefDelay();
		auto coralStationDelay = m_autonSelector->GetCoralStationDelay();

		for (PrimitiveParams *param : m_primParams)
		{

			if (param->GetID() == PRIMITIVE_IDENTIFIER::DO_NOTHING_DELAY)
			{
				param->SetStartDelay(startDelay);
				param->SetReefDelay(reefDelay);
				param->SetCoralStationDelay(coralStationDelay);
			}
		}
	}
}