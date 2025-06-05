#include "frc/RobotController.h"
#include "chassis/ChassisConfigMgr.h"
#include "units/velocity.h"
#include "chassis/CommandSwerveDrivetrain.h"
#include "chassis/generated/TunerConstants302.h"
#include "chassis/generated/TunerConstants9998.h"
#include "RobotIdentifier.h"

ChassisConfigMgr *ChassisConfigMgr::m_instance = nullptr;

ChassisConfigMgr *ChassisConfigMgr::GetInstance()
{
    if (ChassisConfigMgr::m_instance == nullptr)
    {
        ChassisConfigMgr::m_instance = new ChassisConfigMgr();
    }
    return ChassisConfigMgr::m_instance;
}
ChassisConfigMgr::ChassisConfigMgr() : m_maxSpeed(0_mps)
{
    // Constructor implementation can be empty or contain initialization logic
}

std::unique_ptr<subsystems::CommandSwerveDrivetrain> ChassisConfigMgr::CreateDrivetrain()
{
    int32_t teamNumber = frc::RobotController::GetTeamNumber();

    RobotIdentifier id = static_cast<RobotIdentifier>(teamNumber);

    switch (id)
    {
    case RobotIdentifier::COMP_BOT_302:
        m_maxSpeed = TunerConstants302::kSpeedAt12Volts;
        return TunerConstants302::CreateDrivetrain();
        break;

    case RobotIdentifier::CHASSIS_BOT_9998:
        m_maxSpeed = TunerConstants9998::kSpeedAt12Volts;
        return TunerConstants9998::CreateDrivetrain();
        break;

    case RobotIdentifier::SIM_BOT_0:
        m_maxSpeed = TunerConstants302::kSpeedAt12Volts;
        return TunerConstants302::CreateDrivetrain();
        break;

    default:
        return nullptr;
        break;
    }
}