package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constructers.BaseRobot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous (name = "Blue Encoder Autonomous", group = "Autonomous")
public class BlueEncoderAutonomous extends LinearOpMode
{

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


        robot.secureCargo();

        robot.raiseCargoLift(4);

        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(5)
                .build();

        //Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d())



        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(myTrajectory);
        sleep(5000);


        /*
        robot.chassisDrive(BaseRobot.DRIVE_SPEED,-10,-10,5);

        robot.chassisDrive(BaseRobot.DRIVE_SPEED,11,-11,5);

        robot.chassisDrive(BaseRobot.DRIVE_SPEED,-16,-16,5);

        robot.raiseCargoLift(24);

        robot.dumpCargo();

        sleep(1000);

        robot.resetBox();

        robot.chassisDrive(BaseRobot.DRIVE_SPEED,4,4,5);

        robot.chassisDrive(BaseRobot.DRIVE_SPEED,10,-10,5);

        robot.chassisDrive(BaseRobot.DRIVE_SPEED,57,57,5);

        robot.lowerCargoLift(24);
         */

    }

}

