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

#include <string>

#include "chassis/definitions/chassis9997/ChassisConfigChassis_9997.h"
#include "chassis/definitions/chassis9998/ChassisConfigChassis_9998.h"
#include "chassis/definitions/chassis9999/ChassisConfigChassis_9999.h"
#include "chassis/definitions/ChassisConfig.h"
#include "chassis/definitions/ChassisConfigMgr.h"
#include "RobotIdentifier.h"
#include "utils/logging/debug/Logger.h"

using namespace std;

ChassisConfigMgr *ChassisConfigMgr::m_instance = nullptr;
ChassisConfigMgr *ChassisConfigMgr::GetInstance()
{
	if (ChassisConfigMgr::m_instance == nullptr)
	{
		ChassisConfigMgr::m_instance = new ChassisConfigMgr();
	}
	return ChassisConfigMgr::m_instance;
}

ChassisConfigMgr::ChassisConfigMgr() : m_config(nullptr)
{
}

void ChassisConfigMgr::InitChassis(RobotIdentifier id)
{
	switch (id)
	{
	/**
	case RobotIdentifier::COMP_BOT_302:
		m_config = new ChassisConfigCompBot_302();
		break;

	**/
	case RobotIdentifier::CHASSIS_BOT_9998:
		m_config = new ChassisConfigChassis_9998();
		break;

	case RobotIdentifier::CHASSIS_BOT_9997:
		m_config = new ChassisConfigChassis_9997();
		break;
	case RobotIdentifier::PRACTICE_BOT_9999:
		m_config = new ChassisConfigChassis_9999();
		break;
	case RobotIdentifier::SIM_BOT_0:
		m_config = new ChassisConfigChassis_9999();
		break;

	default:
		Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("Skipping chassis initialization because of unknown robot id "), string(""), static_cast<int>(id));
		break;
	}

	if (m_config != nullptr)
	{
		m_config->BuildChassis();
		Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT_ONCE, string("Initialization completed for robot "), string(""), static_cast<int>(id));
	}
}
