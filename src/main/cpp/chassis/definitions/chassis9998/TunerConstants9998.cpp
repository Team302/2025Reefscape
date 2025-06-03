#include "chassis/definitions/chassis9998/TunerConstants9998.h"
#include "chassis/CommandSwerveDrivetrain.h"

subsystems::CommandSwerveDrivetrain TunerConstants9998::CreateDrivetrain()
{
    return {DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight};
}
