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

#include "frc/DriverStation.h"
#include "feedback/DriverFeedback.h"
#include "state/RobotState.h"
#include "state/RobotStateChanges.h"
#include "state/IRobotStateChangeSubscriber.h"
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <feedback/LEDStates.h>
#include <frc/DriverStation.h>

#include "teleopcontrol/TeleopControl.h"
#include "configs/MechanismConfigMgr.h"
#include "mechanisms/DragonTale/DragonTale.h"
#include "mechanisms/IntakeManager/IntakeManager.h"
// #include "mechanisms/noteManager/decoratormods/noteManager.h"

using frc::DriverStation;

DriverFeedback *DriverFeedback::m_instance = nullptr;

DriverFeedback *DriverFeedback::GetInstance()
{
    if (DriverFeedback::m_instance == nullptr)
    {
        DriverFeedback::m_instance = new DriverFeedback();
    }
    return DriverFeedback::m_instance;
}

void DriverFeedback::UpdateFeedback()
{
    UpdateRumble();
    UpdateDiagnosticLEDs();
    UpdateLEDStates();
    CheckControllers();
}

void DriverFeedback::UpdateRumble()
{
    auto controller = TeleopControl::GetInstance();
    if (!frc::DriverStation::IsTeleop())
    {
        controller->SetRumble(0, false, false);
        controller->SetRumble(1, false, false);
    }
}

void DriverFeedback::UpdateLEDStates()
{
    auto mechanismConfigMgr = MechanismConfigMgr::GetInstance()->GetCurrentConfig();
    StateMgr *taleStateManager = mechanismConfigMgr != nullptr ? mechanismConfigMgr->GetMechanism(MechanismTypes::DRAGON_TALE) : nullptr;
    auto taleMgr = taleStateManager != nullptr ? dynamic_cast<DragonTale *>(taleStateManager) : nullptr;
    oldState = currentState;
    if (frc::DriverStation::IsDisabled())
    {
        m_LEDStates->DisabledPattern();
    }
    if (m_climbMode == RobotStateChanges::ClimbMode::ClimbModeOn)
    {
        currentState = DragonLeds::RED;
        if (oldState != currentState)
        {
            m_LEDStates->ResetVariables();
        }
        m_LEDStates->SolidColorPattern(currentState);
    }
    else
    {
        if (oldState != currentState)
        {
            m_LEDStates->ResetVariables();
        }
        if (m_scoringMode == RobotStateChanges::ScoringMode::Coral)
        {
            currentState = DragonLeds::WHITE;
            m_LEDStates->SolidColorPattern(currentState);
        }
        else if (m_scoringMode == RobotStateChanges::ScoringMode::Algae)
        {
            currentState = DragonLeds::AZUL;
            m_LEDStates->SolidColorPattern(currentState);
        }
        if (taleMgr != nullptr)
        {
            if ((taleMgr->GetCurrentState() == taleMgr->STATE_GRAB_ALGAE_REEF) || (taleMgr->GetCurrentState() == taleMgr->STATE_GRAB_ALGAE_FLOOR))
            {
                currentState = DragonLeds::AZUL;
                m_LEDStates->BlinkingPattern(currentState);
            }
            else if (taleMgr->GetCurrentState() == taleMgr->STATE_HUMAN_PLAYER_LOAD)
            {
                currentState = DragonLeds::WHITE;
                m_LEDStates->BlinkingPattern(currentState);
            }
            else if (taleMgr->GetCurrentState() == taleMgr->STATE_HOLD)
            {
                if (taleMgr->GetCoralOutSensorState())
                {
                    currentState = DragonLeds::WHITE;
                    m_LEDStates->BreathingPattern(currentState);
                }
                else if (taleMgr->GetAlgaeSensorState())
                {
                    currentState = DragonLeds::AZUL;
                    m_LEDStates->BreathingPattern(currentState);
                }
                else if (taleMgr->GetCoralOutSensorState() && taleMgr->GetAlgaeSensorState())
                {
                    m_LEDStates->AlternatingColorBlinkingPattern(DragonLeds::WHITE, DragonLeds::AZUL);
                }
            }
            else if (taleMgr->GetCurrentState() == taleMgr->STATE_L1SCORING_POSITION ||
                     taleMgr->GetCurrentState() == taleMgr->STATE_L2SCORING_POSITION ||
                     taleMgr->GetCurrentState() == taleMgr->STATE_L3SCORING_POSITION ||
                     taleMgr->GetCurrentState() == taleMgr->STATE_L4SCORING_POSITION)
            {
                currentState = DragonLeds::WHITE;
                taleMgr->AtTarget() ? m_LEDStates->BlinkingPattern(currentState) : m_LEDStates->SolidColorPattern(currentState); // TODO: add vision alignment to this condition
            }
            else if (taleMgr->GetCurrentState() == taleMgr->STATE_NET ||
                     taleMgr->GetCurrentState() == taleMgr->STATE_PROCESS)
            {
                currentState = DragonLeds::AZUL;
                taleMgr->AtTarget() ? m_LEDStates->BlinkingPattern(currentState) : m_LEDStates->SolidColorPattern(currentState);
            }
        }
    }
}

void DriverFeedback::UpdateDiagnosticLEDs()
{

    if (MechanismConfigMgr::GetInstance()->GetCurrentConfig() != nullptr)
    {
        StateMgr *taleStateManager = MechanismConfigMgr::GetInstance()->GetCurrentConfig()->GetMechanism(MechanismTypes::DRAGON_TALE);
        StateMgr *intakeStateManager = MechanismConfigMgr::GetInstance()->GetCurrentConfig()->GetMechanism(MechanismTypes::INTAKE_MANAGER);
        auto taleMgr = taleStateManager != nullptr ? dynamic_cast<DragonTale *>(taleStateManager) : nullptr;
        auto intakeMgr = intakeStateManager != nullptr ? dynamic_cast<IntakeManager *>(intakeStateManager) : nullptr;

        if (taleMgr != nullptr && intakeMgr != nullptr)
        {
            if (DragonVision::GetDragonVision() != nullptr)
            {
                // auto vision = DragonVision::GetDragonVision();
                // vision->HealthCheck(RobotElementNames::CAMERA_USAGE::LAUNCHE);
                bool questStatus = false;
                bool ll1Status = false;
                bool ll2Status = false;
                bool pigeonfaults = false;
                bool coralInSensor = taleMgr->GetCoralInSensorState();
                bool coralOutSensor = taleMgr->GetCoralOutSensorState();
                bool algaeSensor = taleMgr->GetAlgaeSensorState();
                bool intsakeSensor = intakeMgr->GetIntakeSensorState();
                m_LEDStates->DiagnosticPattern(FMSData::GetInstance()->GetAllianceColor(), coralInSensor, coralOutSensor, algaeSensor, intsakeSensor, questStatus, ll1Status, ll2Status, pigeonfaults);
            }
        }
    }
}

void DriverFeedback::ResetRequests(void)
{
}

DriverFeedback::DriverFeedback() : IRobotStateChangeSubscriber()
{

    RobotState *RobotStates = RobotState::GetInstance();
    RobotStates->RegisterForStateChanges(this, RobotStateChanges::StateChange::DesiredScoringMode_Int);
    RobotStates->RegisterForStateChanges(this, RobotStateChanges::StateChange::ClimbModeStatus_Int);
}
void DriverFeedback::Update(RobotStateChanges::StateChange change, int value)
{
    if (RobotStateChanges::StateChange::ClimbModeStatus_Int == change)
        m_climbMode = static_cast<RobotStateChanges::ClimbMode>(value);

    else if (RobotStateChanges::StateChange::DesiredScoringMode_Int == change)
        m_scoringMode = static_cast<RobotStateChanges::ScoringMode>(value);
}

void DriverFeedback::CheckControllers()
{
    if (m_controllerCounter == 0)
    {
        auto table = nt::NetworkTableInstance::GetDefault().GetTable("XBOX Controller");
        for (auto i = 0; i < DriverStation::kJoystickPorts; ++i)
        {
            table.get()->PutBoolean(std::string("Controller") + std::to_string(i), DriverStation::GetJoystickIsXbox(i));
        }
    }
    m_controllerCounter++;
    if (m_controllerCounter > 25)
    {
        m_controllerCounter = 0;
    }
}
