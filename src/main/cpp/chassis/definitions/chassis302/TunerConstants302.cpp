#include "chassis/definitions/chassis302/TunerConstants302.h"
#include "chassis/CommandSwerveDrivetrain.h"

subsystems::CommandSwerveDrivetrain TunerConstants302::CreateDrivetrain()
{
    return {DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight};
}
