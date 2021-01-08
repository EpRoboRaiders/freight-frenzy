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
import org.firstinspires.ftc.teamcode.constructors.AutonomousTemplate;
import org.firstinspires.ftc.teamcode.constructors.CPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Vector;

@Autonomous(name = "NewLeftBlue", group = "Autonomous")
public class NewLeftBlue extends AutonomousBase {
    private AutonomousTemplate robot = new AutonomousTemplate();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Camera initializing; please wait (~5 seconds)");
        telemetry.update();
        sleep(5000);
        
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        waitForStart();
        robot.ringShooter.teleOpInit();

        telemetry.addData("Analysis", robot.webcam.pipeline.getAnalysis());
        telemetry.addData("Position", robot.webcam.pipeline.getRingAmount());
        telemetry.update();

        sleep(500);

        if (robot.webcam.pipeline.getRingAmount() == CPipeline.RingPosition.NONE) {
            // Placement A
            positionC();
            //shooterPosition();
        } else if (robot.webcam.pipeline.getRingAmount() == CPipeline.RingPosition.ONE) {
            // Placement B
            positionB();
            //shooterPosition();
        } else if (robot.webcam.pipeline.getRingAmount() == CPipeline.RingPosition.FOUR) {
            // Placement C (Default)
            positionC();
            //shooterPosition();
        }

    }

    public void positionA() {

        ArrayList<Trajectory> trajectories = new ArrayList<Trajectory>();
        trajectories.add(robot.drive.trajectoryBuilder(new Pose2d())
                //first trajectory moves the first wobble goal into box A
                //every time the robot stops, a new trajectory must be made
                .lineToConstantHeading(new Vector2d(75, 9))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //second trajectory moves the robot to line up with the second wobble goal
                .lineToLinearHeading(new Pose2d(16, -46, Math.toRadians(90)))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //moves forward to grab the wobble goal
                //.forward(14)
                .lineToConstantHeading(new Vector2d(16, -32))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //moves the wobble goal into box A
                .lineToLinearHeading(new Pose2d(65, 9, Math.toRadians(0)))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //pulls away from the wobble goal and out of the box
                // .back(15)
                .lineToConstantHeading(new Vector2d(49, 9))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //pulls away from the wobble goal and out of the box
                // .back(15)
                .lineToConstantHeading(new Vector2d(55, -11))
                .build());


        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //lines up with the ring goal so it can score and park
                .lineToConstantHeading(new Vector2d(61, -24))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //lines up with the ring goal so it can score and park
                .lineToConstantHeading(new Vector2d(61, -30))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //lines up with the ring goal so it can score and park
                .lineToConstantHeading(new Vector2d(61, -36))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                // .forward(8)
                .lineToConstantHeading(new Vector2d(69, -36))
                .build());

        robot.drive.followTrajectory(trajectories.get(0));
        robot.drive.followTrajectory(trajectories.get(1));
        robot.drive.followTrajectory(trajectories.get(2));
        robot.drive.followTrajectory(trajectories.get(3));
        robot.drive.followTrajectory(trajectories.get(4));
        robot.drive.followTrajectory(trajectories.get(5));
        robot.drive.followTrajectory(trajectories.get(6));

        robot.ringIntake.extendIntake();
        robot.ringShooter.powerShot();

        robot.drive.followTrajectory(trajectories.get(7));

        robot.ringShooter.ringShoot(.55);

        robot.drive.followTrajectory(trajectories.get(8));

        robot.ringShooter.powerShot();
        robot.ringShooter.powerShot();

        robot.ringIntake.retractIntake();

        robot.drive.followTrajectory(trajectories.get(9));



        /*
        for (int i = 0; i < trajectories.size(); i++) {
            if (i == 6) {
                shooterPosition();
            }
            robot.drive.followTrajectory(trajectories.get(i));
        }

         */

    }

    public void positionB() {


        ArrayList<Trajectory> trajectories = new ArrayList<Trajectory>();
        trajectories.add(robot.drive.trajectoryBuilder(new Pose2d())
                //first trajectory moves the robot towards box B
                // .forward(60)
                .lineToConstantHeading(new Vector2d(60, 0))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //second trajectory moves the robot into the box
                .lineToConstantHeading(new Vector2d(95, -14))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //third trajectory moves the robot to line up with the second wobble goal
                .lineToLinearHeading(new Pose2d(14, -50, Math.toRadians(90)))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //moves forward to grab the wobble goal
                //.forward(14)
                .lineToConstantHeading(new Vector2d(13, -36))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //moves the second wobble goal away form the ring
                .lineToLinearHeading(new Pose2d(30, 4, Math.toRadians(0)))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //moves the wobble goal into the box
                .lineToConstantHeading(new Vector2d(87, -14 ))
                .build());


        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //lines up with the ring goal so it can score and park
                .lineToConstantHeading(new Vector2d(75, -18))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //lines up with the ring goal so it can score and park
                .lineToConstantHeading(new Vector2d(60, -33))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //lines up with the ring goal so it can score and park
                .lineToConstantHeading(new Vector2d(60, -40))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //lines up with the ring goal so it can score and park
                .lineToConstantHeading(new Vector2d(60, -46))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                // .forward(8)
                .lineToConstantHeading(new Vector2d(68, -46))
                .build());

        robot.drive.followTrajectory(trajectories.get(0));
        robot.drive.followTrajectory(trajectories.get(1));
        robot.drive.followTrajectory(trajectories.get(2));
        robot.drive.followTrajectory(trajectories.get(3));
        robot.drive.followTrajectory(trajectories.get(4));
        robot.drive.followTrajectory(trajectories.get(5));
        robot.drive.followTrajectory(trajectories.get(6));
        robot.drive.followTrajectory(trajectories.get(7));


        robot.ringIntake.extendIntake();
        robot.ringShooter.ringShoot(.6);

        robot.drive.followTrajectory(trajectories.get(8));

        robot.ringShooter.ringShoot(.6);

        robot.drive.followTrajectory(trajectories.get(9));

        robot.ringShooter.ringShoot(.57);
        robot.ringShooter.powerShot();

        robot.ringIntake.retractIntake();

        robot.drive.followTrajectory(trajectories.get(10));

        /*
        for (int i = 0; i < trajectories.size(); i++) {

            robot.drive.followTrajectory(trajectories.get(i));

            if(i == 6) {
                shooterPosition();
            }

             */


    }

    public void positionC() {

        ArrayList<Trajectory> trajectories = new ArrayList<Trajectory>();
        trajectories.add(robot.drive.trajectoryBuilder(new Pose2d())
                //first trajectory moves the first wobble goal into box C
                //every time the robot stops, a new trajectory must be made
                .lineToConstantHeading(new Vector2d(120, 9))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size() - 1).end())
                //second trajectory moves the robot to line up with the second wobble goal
                .lineToLinearHeading(new Pose2d(19, -46, Math.toRadians(90)))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size() - 1).end())
                //moves forward to grab the wobble goal
                //.forward(14)
                .lineToConstantHeading(new Vector2d(19, -32))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size() - 1).end())
                //moves the wobble goal into box C
                .lineToLinearHeading(new Pose2d(30, 4, Math.toRadians(0)))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size() - 1).end())
                .lineToConstantHeading(new Vector2d(115, 4))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size() - 1).end())
                //pulls away from the wobble goal and out of the box
                .lineToConstantHeading(new Vector2d(105, 4))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size() - 1).end())
                //lines up with the ring goal so it can score and park
                .lineToConstantHeading(new Vector2d(66, -16))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size() - 1).end())
                //lines up with the ring goal so it can score and park
                .lineToConstantHeading(new Vector2d(66, -22))
                .build());



        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size() - 1).end())
                //lines up with the ring goal so it can score and park
                .lineToConstantHeading(new Vector2d(66, -28))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size() - 1).end())
                //lines up with the ring goal so it can score and park
                .lineToConstantHeading(new Vector2d(66, -34))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size() - 1).end())
                //lines up with the ring goal so it can score and park
                .lineToConstantHeading(new Vector2d(66, -40))
                .build());







        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size() - 1).end())
                //moves forward to park
                .lineToConstantHeading(new Vector2d(74, -40))
                .build());

        robot.drive.followTrajectory(trajectories.get(0));
        robot.drive.followTrajectory(trajectories.get(1));
        robot.drive.followTrajectory(trajectories.get(2));
        robot.drive.followTrajectory(trajectories.get(3));
        robot.drive.followTrajectory(trajectories.get(4));
        robot.drive.followTrajectory(trajectories.get(5));
        robot.drive.followTrajectory(trajectories.get(6));
        robot.drive.followTrajectory(trajectories.get(7));


        robot.ringIntake.extendIntake();
        robot.ringShooter.ringShoot(.57);

        robot.drive.followTrajectory(trajectories.get(8));

        robot.ringShooter.ringShoot(.57);

        robot.drive.followTrajectory(trajectories.get(9));

        robot.ringShooter.ringShoot(.57);
        robot.ringShooter.powerShot();

        robot.ringIntake.retractIntake();

        robot.drive.followTrajectory(trajectories.get(10));

        /*
        for (int i = 0; i < trajectories.size(); i++) {
            if (i == 7) {
                shooterPosition();
            }
            robot.drive.followTrajectory(trajectories.get(i));
        }

         */
    }

    public void shooterPosition() {

        /*
        ArrayList<Trajectory> trajectories = new ArrayList<Trajectory>();

        trajectories.add(robot.drive.trajectoryBuilder(new Pose2d())
                //first trajectory moves the first wobble goal into box C
                //every time the robot stops, a new trajectory must be made
                .strafeRight(6)
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size() - 1).end())
                //second trajectory moves the robot to line up with the second wobble goal
                .strafeRight(6)
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size() - 1).end())
                //moves forward to grab the wobble goal
                //.forward(14)
                .strafeRight(6)
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size() - 1).end())
                //moves the wobble goal into box C
                .forward(8)
                .build());

        robot.ringIntake.extendIntake();

        robot.drive.followTrajectory(trajectories.get(0));

        robot.ringShooter.powerShot();

        robot.drive.followTrajectory(trajectories.get(1));

        robot.ringShooter.powerShot();

        robot.drive.followTrajectory(trajectories.get(2));

        robot.ringShooter.powerShot();

        robot.drive.followTrajectory(trajectories.get(3));
        
        //robot.ringShooter.autonomousTowerShot();
        
        robot.ringIntake.retractIntake();

         */


        
        /*
        robot.ringShooter.towerShot();
        robot.hopperLifter.setPosition(0.09);

        sleep(500);

        robot.intakeArm.setPower(0);
        robot.leftShooter.setPower(.71);
        robot.rightShooter.setPower(.71);
        robot.shooterArm.setPosition(1);

        sleep(500);

        robot.leftShooter.setPower(0);
        robot.rightShooter.setPower(0);
        robot.shooterArm.setPosition(.75);

        sleep(100);
        robot.hopperLifter.setPosition(0.04);

        sleep(1000);

        robot.leftShooter.setPower(.71);
        robot.rightShooter.setPower(.71);
        robot.shooterArm.setPosition(1);

        sleep(500);

        robot.leftShooter.setPower(0);
        robot.rightShooter.setPower(0);
        robot.shooterArm.setPosition(.8);

        sleep(100);
        robot.hopperLifter.setPosition(0);

        sleep(1000);

        robot.leftShooter.setPower(.71);
        robot.rightShooter.setPower(.71);
        robot.shooterArm.setPosition(1);

        sleep(500);

        robot.leftShooter.setPower(0);
        robot.rightShooter.setPower(0);
        robot.shooterArm.setPosition(.8);
        robot.intakeArm.setPower(.6);

        sleep(1500);

        robot.intakeArm.setPower(0);
        robot.hopperLifter.setPosition(0.15);

         */

    }
}
