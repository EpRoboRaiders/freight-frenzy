package org.firstinspires.ftc.teamcode.autonomous.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "TestRoadrunner", group = "RoadrunnerTest")
public class RoadrunnerTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Pose2d startPose = new Pose2d(-63, 48, Math.toRadians(0));

        // drive.setPoseEstimate(startPose);

        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d())
                .splineToConstantHeading(new Vector2d(64, 12), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(48, 12), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(72, -12), Math.toRadians(0))

                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(myTrajectory);
    }



}
