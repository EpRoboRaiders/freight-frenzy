package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.drive.Drive;
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

        Trajectory myTrajectory1 = drive.trajectoryBuilder(new Pose2d())
                .back(5)
                .build();

        double turn1 = Math.toRadians(12);
        Trajectory myTrajectory2 = drive.trajectoryBuilder(myTrajectory1.end().plus(new Pose2d(0, 0, turn1)), false)
                .back(40)
                .build();

        double turn2 = Math.toRadians(-103);
        Trajectory myTrajectory3 = drive.trajectoryBuilder(myTrajectory2.end().plus(new Pose2d(0, 0, turn2)), false)
                .back(24)
                .build();

        Trajectory myTrajectory4 = drive.trajectoryBuilder(myTrajectory3.end())
                .forward(34)
                .build();

        double turn3 = Math.toRadians(90);
        Trajectory myTrajectory5 = drive.trajectoryBuilder(myTrajectory4.end().plus(new Pose2d(0, 0, turn3)), false)
                .forward(42)
                .build();

        double turn4 = Math.toRadians(10);
        Trajectory myTrajectory6 = drive.trajectoryBuilder(myTrajectory5.end().plus(new Pose2d(0, 0, turn4)), false)
                .back(21)
                .build();

        double turn5 = Math.toRadians(-10);
        Trajectory myTrajectory7 = drive.trajectoryBuilder(myTrajectory6.end().plus(new Pose2d(0, 0, turn5)), false)
                .back(30)
                .build();



        waitForStart();

        robot.secureCargo();

        double liftheight = robot.getDuckPosition();

        telemetry.addData("TSE position", robot.duckpostionname());
        telemetry.update();

        drive.followTrajectory(myTrajectory1);

        drive.turn(turn1);

        drive.followTrajectory(myTrajectory2);

        drive.turn(turn2);

        drive.followTrajectory(myTrajectory3);

        robot.raiseCargoLift(liftheight);

        while (opModeIsActive() && !robot.isLiftFinished()) {
            robot.update();
        }

        sleep(100);

        robot.dumpCargo();
        sleep(1000);

        drive.followTrajectory(myTrajectory4);

        drive.turn(turn3);

        drive.followTrajectory(myTrajectory5);

        robot.startReverseSpinnerSpinner();
        sleep(3700);

        robot.stopCarouselSpinnerSpinner();

        drive.turn(turn4);

        drive.followTrajectory(myTrajectory6);

        drive.turn(turn5);

        robot.secureCargo();

        robot.lowerCargoLift(liftheight - 2);

        while (opModeIsActive() && !robot.isLiftFinished()) {
            robot.update();
        }
    }
}
