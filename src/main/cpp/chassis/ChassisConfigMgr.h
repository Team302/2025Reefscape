#pragma once

#include "chassis/generated/TunerConstants302.h"
#include "chassis/generated/TunerConstants9998.h"

using namespace ctre::phoenix6;

namespace subsystems
{
    class CommandSwerveDrivetrain; // Forward declaration
}

class ChassisConfigMgr
{
public:
    static ChassisConfigMgr *GetInstance();

    std::unique_ptr<subsystems::CommandSwerveDrivetrain> CreateDrivetrain();

    units::meters_per_second_t GetMaxSpeed() { return m_maxSpeed; }

private:
    ChassisConfigMgr();
    ~ChassisConfigMgr() = default;

    static ChassisConfigMgr *m_instance;
    units::meters_per_second_t m_maxSpeed;
};