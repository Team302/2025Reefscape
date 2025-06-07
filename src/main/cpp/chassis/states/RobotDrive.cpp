#include "chassis/states/RobotDrive.h"
#include "utils/logging/debug/Logger.h"

// Note the simplified constructor and AddRequirements call
RobotDrive::RobotDrive(subsystems::CommandSwerveDrivetrain *chassis,
                       TeleopControl *controller,
                       units::velocity::meters_per_second_t maxSpeed,
                       units::angular_velocity::degrees_per_second_t maxAngularRate) : m_chassis(chassis),
                                                                                       m_controller(controller),
                                                                                       m_maxSpeed(maxSpeed),
                                                                                       m_maxAngularRate(maxAngularRate)
{
    AddRequirements(m_chassis);
}

void RobotDrive::Execute()
{
    double forward = m_controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_FORWARD);
    double strafe = m_controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_STRAFE);
    double rotate = m_controller->GetAxisValue(TeleopControlFunctions::HOLONOMIC_DRIVE_ROTATE);

    m_chassis->SetControl(
        m_RobotDriveRequest.WithVelocityX(forward * m_maxSpeed)
            .WithVelocityY(strafe * m_maxSpeed)
            .WithRotationalRate(rotate * m_maxAngularRate));
}

bool RobotDrive::IsFinished()
{
    // A default drive command should never finish on its own.
    // It runs until it is interrupted by another command.
    return false;
}

void RobotDrive::End(bool interrupted)
{
    m_chassis->ApplyRequest([]() -> auto
                            { return swerve::requests::SwerveDriveBrake{}; });
}