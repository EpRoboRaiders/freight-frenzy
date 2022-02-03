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

        Trajectory myTrajectory1 = drive.trajectoryBuilder(new Pose2d())
                .back(0.1)
                .build();

        double turn1 = Math.toRadians(-45);
        Trajectory myTrajectory2 = drive.trajectoryBuilder(myTrajectory1.end().plus(new Pose2d(0, 0, turn1)), false)
                .back(25)
                .build();

        Trajectory myTrajectory3 = drive.trajectoryBuilder(myTrajectory2.end())
                .back(2)
                .build();

        Trajectory myTrajectory4 = drive.trajectoryBuilder(myTrajectory3.end())
                .forward(5)
                .build();

        double turn3 = Math.toRadians(-10);
        Trajectory myTrajectory5 = drive.trajectoryBuilder(myTrajectory4.end().plus(new Pose2d(0, 0, turn3)), false)
                .forward(5)
                .build();

        double turn4 = Math.toRadians(-60);
        Trajectory myTrajectory6 = drive.trajectoryBuilder(myTrajectory5.end().plus(new Pose2d(0, 0, turn4)), false)
                .forward(75)
                .build();


        waitForStart();

        robot.secureCargo();

        double liftheight = robot.getDuckPositionblue();

        telemetry.addData("TSE position", robot.duckpostionname());
        telemetry.update();

        drive.followTrajectory(myTrajectory1);

        drive.turn(turn1);

        drive.followTrajectory(myTrajectory2);

        drive.followTrajectory(myTrajectory3);

        robot.raiseCargoLift(liftheight);

        while (opModeIsActive() && !robot.isLiftFinished()) {
            robot.update();
        }

        robot.dumpCargo();

        sleep(1000);

        drive.followTrajectory(myTrajectory4);

        drive.turn(turn3);

        drive.followTrajectory(myTrajectory5);

        drive.turn(turn4);

        drive.followTrajectory(myTrajectory6);

        robot.secureCargo();
        sleep(1000);

        robot.lowerCargoLift(liftheight);

        while (opModeIsActive() && !robot.isLiftFinished()) {
            robot.update();
        }



    }
}
