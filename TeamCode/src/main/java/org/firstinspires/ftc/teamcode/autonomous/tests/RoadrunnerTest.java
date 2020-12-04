package org.firstinspires.ftc.teamcode.autonomous.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "TestRoadrunner", group = "RoadrunnerTest")
public class RoadrunnerTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //from the roadrunner example they wanted their robot to be in a certain starting position
        //Pose2d startPose = new Pose2d(-63, 48, Math.toRadians(0));

        //drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                //first trajectory moves the first wobble goal into box A
                //every time the robot stops, a new trajectory must be made
                .splineToConstantHeading(new Vector2d(72, 9), Math.toRadians(0))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                //second trajectory moves the robot back to get the second wobble goal
                //turning 90 degrees while moving back so that the robot lines up with the wobble goal
                .back(20)
                .build();


        waitForStart();
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
    }



}
