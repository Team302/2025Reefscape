$$_COPYRIGHT_$$
$$_GEN_NOTICE_$$

#pragma once

#include <string>
#include <memory>

// FRC Includes
#include <networktables/NetworkTable.h>

$$_INCLUDE_FILES_$$

#include "mechanisms/base/BaseMech.h"
#include "mechanisms/base/StateMgr.h"
#include "mechanisms/controllers/ControlData.h"

#include "configs/RobotElementNames.h"
#include "configs/RobotConfigMgr.h"

class $$_MECHANISM_INSTANCE_NAME_$$ : public BaseMech _STATE_MANAGER_START_, public StateMgr _STATE_MANAGER_END_
{
public:
    enum STATE_NAMES
    {
        $$_STATE_NAMES_$$
    };

    $$_MECHANISM_INSTANCE_NAME_$$(RobotConfigMgr::RobotIdentifier activeRobotId);
    $$_MECHANISM_INSTANCE_NAME_$$() = delete;
    ~$$_MECHANISM_INSTANCE_NAME_$$() = default;

    $$_CREATE_FUNCTIONS_$$
    $$_PUBLIC_INITIALZATION_FUNCTIONS_$$

    _STATE_MANAGER_START_
    /// @brief Set the control constants (e.g. PIDF values).
    /// @param indentifier the motor controller usage to identify the motor
    /// @param slot position on the motor controller to set
    /// @param pid control data / constants
    virtual void SetControlConstants(RobotElementNames::MOTOR_CONTROLLER_USAGE indentifier, int slot, ControlData pid);

    /// @brief update the output to the mechanism using the current controller and target value(s)
    virtual void Update();

    $$_TARGET_UPDATE_FUNCTIONS_$$

    $$_PID_UPDATE_FUNCTIONS_$$

    _MECHANISM_HAS_SOLENOIDS_START_
    /// @brief Set the target value for the actuator
    /// @param identifier solenoid Usage to indicate what motor to update
    /// @param extend target value
    virtual void UpdateTarget(RobotElementNames::SOLENOID_USAGE identifier, bool extend);
    virtual bool IsAtMinPosition(RobotElementNames::SOLENOID_USAGE identifier) const;
    virtual bool IsAtMaxPosition(RobotElementNames::SOLENOID_USAGE identifier) const;
    _MECHANISM_HAS_SOLENOIDS_END_

    _MECHANISM_HAS_MOTORS_START_
    virtual bool IsAtMinPosition(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier) const;
    virtual bool IsAtMaxPosition(RobotElementNames::MOTOR_CONTROLLER_USAGE identifier) const;
    _MECHANISM_HAS_MOTORS_END_
    _STATE_MANAGER_END_

    void CreateAndRegisterStates();
    void Cyclic();
    void RunCommonTasks() override;

    RobotConfigMgr::RobotIdentifier getActiveRobotId() { return m_activeRobotId; }

    $$_MECHANISM_ELEMENTS_GETTERS_$$

    static std::map<std::string, STATE_NAMES> stringToSTATE_NAMESEnumMap;

protected:
    RobotConfigMgr::RobotIdentifier m_activeRobotId;
    std::string m_ntName;
    std::string m_tuningIsEnabledStr;
    bool m_tuning = false;
    std::shared_ptr<nt::NetworkTable> m_table;

    void SetCurrentState(int state, bool run) override;
    ControlData *GetControlData(std::string name) override;

private:
    std::unordered_map<std::string, STATE_NAMES> m_stateMap;

    $$_MECHANISM_ELEMENTS_$$

    $$_TUNABLE_PARAMETERS_$$

    void CheckForTuningEnabled();
    void ReadTuningParamsFromNT();
    void PushTuningParamsToNT();

    $$_PRIVATE_INITIALZATION_FUNCTIONS_$$

    $$_TARGET_MEMBER_VARIABLES_$$
};