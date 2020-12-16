package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.constructors.AttachmentTemplate;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Vector;

@Autonomous(name = "NewLeftBlue", group = "Autonomous")
public class NewLeftBlue extends AutonomousBase {
    private SampleMecanumDrive drive = null;
    private AttachmentTemplate robot = new AttachmentTemplate();

    @Override
    public void runOpMode() {

        drive = new SampleMecanumDrive(hardwareMap);
        robot.init(hardwareMap);
        cameraInit();
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        waitForStart();
        robot.hopperLifter.setPosition(0.15);
        robot.shooterArm.setPosition(.8);

        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Position", pipeline.position);
        telemetry.update();

        sleep(2000);

        if (pipeline.position == RingStackMeasurerPipeline.RingPosition.NONE) {
            // Placement A2
            positionA();
            //shooterPosition();
        } else if (pipeline.position == RingStackMeasurerPipeline.RingPosition.ONE) {
            // Placement B
            positionB();
            //shooterPosition();
        } else if (pipeline.position == RingStackMeasurerPipeline.RingPosition.FOUR) {
            // Placement C (Default)
            positionC();
            //shooterPosition();
        }

    }


    public void cameraInit() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new RingStackMeasurerPipeline();
        webcam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        //webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
        });
        telemetry.addData("Status", "Camera initializing; please wait (~5 seconds)");
        telemetry.update();
        sleep(5000);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        waitForStart();
    }

    public void positionA() {
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                //first trajectory moves the first wobble goal into box A
                //every time the robot stops, a new trajectory must be made
                .lineToConstantHeading(new Vector2d(74, 9))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                //second trajectory moves the robot to line up with the second wobble goal
                .lineToLinearHeading(new Pose2d(15, -46, Math.toRadians(90)))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                //moves forward to grab the wobble goal
                .forward(14)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                //moves the wobble goal into box A
                .lineToLinearHeading(new Pose2d(64, 9, Math.toRadians(0)))
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                //pulls away from the wobble goal and out of the box
                .back(15)
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                //lines up with the ring goal so it can score and park
                .lineToConstantHeading(new Vector2d(61, -16))
                .build();

        //insert shooting

        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .forward(8)
                .build();

        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
        drive.followTrajectory(traj5);
        drive.followTrajectory(traj6);

        shooterPosition();

        drive.followTrajectory(traj7);
    }

    public void positionB() {
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
                .lineToLinearHeading(new Pose2d(6, -50, Math.toRadians(90)))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                //moves forward to grab the wobble goal
                .forward(15)
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                //moves the second wobble goal away form the ring
                .lineToLinearHeading(new Pose2d(26, 6, Math.toRadians(0)))
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                //moves the wobble goal into the box
                .lineToConstantHeading(new Vector2d(80, -18 ))
                .build();

        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                //lines up with the ring goal so it can score and park
                .lineToConstantHeading(new Vector2d(50, -24))
                .build();

        //insert shooting

        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                //parks on the white line
                .forward(8)
                .build();

        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
        drive.followTrajectory(traj5);
        drive.followTrajectory(traj6);
        drive.followTrajectory(traj7);
        drive.followTrajectory(traj8);
    }

    public void positionC() {
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
                .forward(44)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                //moves the wobble goal into box C
                .lineToLinearHeading(new Pose2d(110, 9, Math.toRadians(0)))
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                //pulls away from the wobble goal and out of the box
                .back(15)
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                //lines up with the ring goal so it can score and park
                .lineToConstantHeading(new Vector2d(61, -16))
                .build();

        //insert shooting

        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
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
    }

    public void shooterPosition() {

        robot.intakeArm.setPower(-.4);
        robot.hopperLifter.setPosition(0.09);

        sleep(500);

        robot.intakeArm.setPower(0);
        robot.leftShooter.setPower(.75);
        robot.rightShooter.setPower(.75);
        robot.shooterArm.setPosition(1);

        sleep(500);

        robot.leftShooter.setPower(0);
        robot.rightShooter.setPower(0);
        robot.shooterArm.setPosition(.75);
        robot.hopperLifter.setPosition(0.05);

        sleep(500);

        robot.leftShooter.setPower(.75);
        robot.rightShooter.setPower(.75);
        robot.shooterArm.setPosition(1);

        sleep(500);

        robot.leftShooter.setPower(0);
        robot.rightShooter.setPower(0);
        robot.shooterArm.setPosition(.8);
        robot.hopperLifter.setPosition(0);

        sleep(500);

        robot.leftShooter.setPower(.70);
        robot.rightShooter.setPower(.70);
        robot.shooterArm.setPosition(1);

        sleep(500);

        robot.leftShooter.setPower(0);
        robot.rightShooter.setPower(0);
        robot.shooterArm.setPosition(.8);
        robot.intakeArm.setPower(.6);

        sleep(1500);

        robot.intakeArm.setPower(0);

    }
}
