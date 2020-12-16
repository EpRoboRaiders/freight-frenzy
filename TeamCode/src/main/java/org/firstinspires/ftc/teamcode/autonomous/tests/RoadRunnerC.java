package org.firstinspires.ftc.teamcode.autonomous.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "RoadRunnerC", group = "RoadrunnerTest")
public class RoadRunnerC extends LinearOpMode {

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                //first trajectory moves the first wobble goal into box C
                //every time the robot stops, a new trajectory must be made
                .lineToConstantHeading(new Vector2d(120, 9))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                //second trajectory moves the robot to line up with the second wobble goal
                //18 is actually 15 for the x coordinate
                .lineToLinearHeading(new Pose2d(18.5, -46, Math.toRadians(90)))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                //moves forward to grab the wobble goal
                .forward(15)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                //moves the wobble goal into box C
                .lineToLinearHeading(new Pose2d(30, 4, Math.toRadians(0)))
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .forward(90)
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                //pulls away from the wobble goal and out of the box
                .back(15)
                .build();

        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                //lines up with the ring goal so it can score and park
                .lineToConstantHeading(new Vector2d(61, -16))
                .build();

        //insert shooting

        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                //moves forward to park
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