package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constructers.BaseRobot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "R2 Encoder Autonomous", group = "Autonomous")
public class R2EncoderAutonomous extends LinearOpMode {
    private BaseRobot robot = new BaseRobot();


    /**
     * Override this method and place your code here.
     * <p>
     * Please do not swallow the InterruptedException, as it is used in cases
     * where the op mode needs to be terminated early.
     *
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.init(hardwareMap);

        waitForStart();

        robot.secureCargo();

        //moves forward around the ducks.
        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d())
                .back(42)
                .build();

        //moves forward to drop off the pre-loaded box
        Trajectory myTrajectory2 = drive.trajectoryBuilder(myTrajectory.end().plus(new Pose2d(0, 0, Math.toRadians(-90))), false)
                .back(28)
                .build();

        //pulls back into the wall
        Trajectory myTrajectory3 = drive.trajectoryBuilder(myTrajectory2.end())
                .forward(33)
                .build();

        //moves forward to use the carousel
        Trajectory myTrajectory4 = drive.trajectoryBuilder(myTrajectory3.end())
                .back(15)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        //traj 1
        drive.followTrajectory(myTrajectory);
        /*turns*/ drive.turn(Math.toRadians(-90));
        sleep(500);
        //traj 2
        drive.followTrajectory(myTrajectory2);
        //drops preloaded box
        robot.raiseCargoLift(26.5);
        robot.dumpCargo();
        sleep(1000);
        robot.resetBox();
        sleep(500);
        //traj 3
        drive.followTrajectory(myTrajectory3);
        /*turns*/ drive.turn(Math.toRadians(-90));
        sleep(5000);
        //traj 4
        drive.followTrajectory(myTrajectory4);
        //uses carousel


        /*
        robot.init(hardwareMap);


        waitForStart();

        robot.secureCargo();

        //Starts under the Carousel
        robot.chassisDrive(BaseRobot.DRIVE_SPEED, -35, -35, 5);

        //Turns to line up with the Shipping Hub
        robot.chassisDrive(BaseRobot.DRIVE_SPEED, 22, -22, 5); //19 inches is about a 90 degree turn

        //Drives forward to drop off the pre-loaded box
        robot.chassisDrive(BaseRobot.DRIVE_SPEED, -26, -26, 5);

        //Drops off the pre-loaded box
        robot.raiseCargoLift(28);
        robot.dumpCargo();
        sleep(1000);
        robot.resetBox();

        //Pulls back to line up with the carousel
        robot.chassisDrive(BaseRobot.DRIVE_SPEED, 20, 20, 5);

        //Slows down before hitting wall
        robot.chassisDrive(BaseRobot.SLOW_SPEED, 13, 13, 5);

        //Turns to line up with the carousel
        robot.chassisDrive(BaseRobot.DRIVE_SPEED, 19, -19, 5);

        //Moves forward to go to the carousel
        robot.chassisDrive(BaseRobot.DRIVE_SPEED, -36.5, -36.5, 5);

        robot.chassisDrive(BaseRobot.DRIVE_SPEED,25,-25,5);

        robot.chassisDrive(BaseRobot.DRIVE_SPEED,-4.5,-4.5,5);

        robot.startReverseSpinnerSpinner();
        sleep(3500);
        robot.stopCarouselSpinnerSpinner();

        //Lowers the lift back to the ground
        robot.lowerCargoLift(26);

        robot.chassisDrive(BaseRobot.DRIVE_SPEED,19,-19,5);

        //Pulls back into Storage Unit
        robot.chassisDrive(BaseRobot.DRIVE_SPEED, -16, -16, 5);

        robot.chassisDrive(BaseRobot.DRIVE_SPEED,-3,3,5);

        robot.chassisDrive(BaseRobot.DRIVE_SPEED,-7,-7,5);

        robot.chassisDrive(BaseRobot.DRIVE_SPEED,20,-20,5);

        robot.chassisDrive(BaseRobot.DRIVE_SPEED,8,8,5);
         */
    }
}
