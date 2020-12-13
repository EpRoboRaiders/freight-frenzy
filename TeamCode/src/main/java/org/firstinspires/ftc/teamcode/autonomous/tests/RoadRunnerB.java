package org.firstinspires.ftc.teamcode.autonomous.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Vector;

@Autonomous(name = "RoadRunnerB", group = "RoadrunnerTest")
public class RoadRunnerB extends LinearOpMode {

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                //first trajectory moves the robot towards box B
                .forward(60)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                //second trajectory moves the robot into the box
                .lineToConstantHeading(new Vector2d(95, -14))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                //moves the robot out of the box and towards the second wobble goal
                .lineToLinearHeading(new Pose2d(12, -50, Math.toRadians(90)))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                //moves forward to grab the wobble goal
                .forward(15)
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                //moves the second wobble goal away form the ring
                .lineToLinearHeading(new Pose2d(30, 4, Math.toRadians(0)))
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                //moves the wobble goal into the box
                .lineToConstantHeading(new Vector2d(90, -14 ))
                .build();

        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                //lines up with the ring goal so it can score and park
                .lineToConstantHeading(new Vector2d(58, -22))
                .build();

        //insert shooting

        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                //parks on the white line
                .forward(8)
                .build();

        waitForStart();
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
        drive.followTrajectory(traj5);
        drive.followTrajectory(traj6);
        drive.followTrajectory(traj7);
        drive.followTrajectory(traj8);
    }
}
