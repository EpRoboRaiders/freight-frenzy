package org.firstinspires.ftc.teamcode.autonomous.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Position C Test", group = "RoadrunnerTest")
public class PositionCTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-63, 48, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .splineToConstantHeading(new Vector2d(-24, 60), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(52, 60), Math.toRadians(0))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())

                .splineToConstantHeading(new Vector2d(-24, 60), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-60, 48), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-60, 36), Math.toRadians(0))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineToConstantHeading(new Vector2d(-24, 60), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(52, 60), Math.toRadians(0))
                .build();



        waitForStart();
        drive.followTrajectory(traj1);
    }



}
