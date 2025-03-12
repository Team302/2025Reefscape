#include <gtest/gtest.h>
#include <frc/RobotController.h>
#include "chassis/definitions/ChassisConfigMgr.h"
#include "chassis/SwerveChassis.h"
#include "chassis/states/DriveToCoralStation.h"
#include "chassis/states/RobotDrive.h"
#include "chassis/states/TrajectoryDrivePathPlanner.h"
#include "fielddata/DragonTargetFinder.h"
#include "chassis/ChassisOptionEnums.h"
#include "vision/DragonVisionStructLogger.h"

class DriveToCoralStationTest : public ::testing::Test
{
protected:
    RobotDrive *robotDrive;
    TrajectoryDrivePathPlanner *trajectoryDrivePathPlanner;
    DriveToCoralStation *driveToCoralStation;

    void SetUp() override
    {
        int32_t teamNumber = frc::RobotController::GetTeamNumber();
        ChassisConfigMgr::GetInstance()->InitChassis(static_cast<RobotIdentifier>(teamNumber));
        auto chassisConfig = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
        auto chassis = chassisConfig->GetSwerveChassis();
        robotDrive = new RobotDrive(chassis);
        trajectoryDrivePathPlanner = new TrajectoryDrivePathPlanner(robotDrive);
        driveToCoralStation = new DriveToCoralStation(robotDrive, trajectoryDrivePathPlanner);
    }

    void TearDown() override
    {
        delete driveToCoralStation;
        delete trajectoryDrivePathPlanner;
        delete robotDrive;
    }
};

TEST_F(DriveToCoralStationTest, GetDriveStateName)
{
    EXPECT_EQ(driveToCoralStation->GetDriveStateName(), "DriveToCoralStation");
}

TEST_F(DriveToCoralStationTest, WhyDone)
{
    EXPECT_EQ(driveToCoralStation->WhyDone(), "Trajectory isn't finished/Error");
}

TEST_F(DriveToCoralStationTest, IsDone)
{
    //if haven't request to drive anywhere, then should be already done
    EXPECT_TRUE(driveToCoralStation->IsDone());
}

TEST_F(DriveToCoralStationTest, CalcHeadingCorrection)
{
    EXPECT_EQ(driveToCoralStation->CalcHeadingCorrection(0_deg, 0.0, 0.0), 0_deg_per_s);
}


/*TEST_F(DriveToCoralStationTest, CreateTrajectory)
{
    auto targetFinder = DragonTargetFinder::GetInstance();
    auto target = DragonTargetFinderTarget::CLOSEST_CORAL_STATION_MIDDLE;
    auto info = targetFinder->GetPose(target);
    auto result = driveToCoralStation->CreateTrajectory(info);
    auto initPose = result.getInitialPose();
    DragonVisionStructLogger::logPose2d(initPose);
    ASSERT_NE(initPose, frc::Pose2d());
}*/

TEST_F(DriveToCoralStationTest, InitFromTrajectory)
{
    ChassisMovement chassisMovement;
    pathplanner::PathPlannerTrajectory trajectory;
    driveToCoralStation->InitFromTrajectory(chassisMovement, trajectory);
}

TEST_F(DriveToCoralStationTest, GetTotalTrajectoryTime)
{
    EXPECT_EQ(driveToCoralStation->GetTotalTrajectoryTime(), 0_s);
}

