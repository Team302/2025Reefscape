#include "chassis/definitions/TunerConstants9997.h"
#include "chassis/definitions/TunerSwerveDrivetrain9997.h"

TunerSwerveDrive TunerConstants9997::CreateDrivetrain()
{
    return {DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight};
}
