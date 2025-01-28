$$_COPYRIGHT_$$
$$_GEN_NOTICE_$$

#pragma once

#include "configs/MechanismConfig.h"

class MechanismConfigMgr
{
public:
    enum RobotIdentifier
    {
        CHASSISBOT_9998 = 9998,
        CHASSIS_BOT_9997 = 9997,
        $$_ROBOT_CONFIGURATIONS_NAMES_ENUMS_$$

            MAX_ROBOT_IDENTIFIERS
    };

    static MechanismConfigMgr *GetInstance();
    MechanismConfig *GetCurrentConfig() const { return m_config; }
    void InitRobot(RobotIdentifier);

private:
    MechanismConfigMgr();
    ~MechanismConfigMgr() = default;

    static MechanismConfigMgr *m_instance;
    MechanismConfig *m_config;
};
