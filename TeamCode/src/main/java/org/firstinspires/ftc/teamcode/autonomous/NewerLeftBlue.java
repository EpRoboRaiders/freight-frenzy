package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.constructors.AutonomousTemplate;
import org.firstinspires.ftc.teamcode.constructors.CPipeline;
import java.util.ArrayList;


@Autonomous(name = "NewerLeftBlue", group = "Autonomous")
public class NewerLeftBlue extends LinearOpMode {
    private AutonomousTemplate robot = new AutonomousTemplate();

    //private final double POWER_SHOT_TURN_TO_POSITION_A =   0 ; //-10.57;
    //private final double POWER_SHOT_TURN_TO_POSITION_B =  -5 ; // -16.35 + 10.57;
    //private final double POWER_SHOT_TURN_TO_POSITION_C =  -10; //-21.80 + 5;

    private int ringAmount = 0;
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

        ringAmount = robot.tensorCamera.getRingAmount();

        telemetry.addData("Analysis", ringAmount);
        telemetry.update();

        if (ringAmount == 0) {
            // Placement A
            positionA();
            // shooterPosition();
        } else if (ringAmount == 1) {
            // Placement B
            positionB();
            // shooterPosition();
        } else { /*if (ringAmount == 2)*/
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
                .lineToConstantHeading(new Vector2d(71, 12))
                .build());


        // trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
        //pulls away from the wobble goal and out of the box
        //.lineToConstantHeading(new Vector2d(50, 10))
        //.build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //moves to line up with the tower shot
                .lineToConstantHeading(new Vector2d(53, -12))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //second trajectory moves the robot to line up with the second wobble goal
                .lineToLinearHeading(new Pose2d(15, -50, Math.toRadians(-90))) //0
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //moves forward to grab the second wobble goal
                //.forward(14)
                .lineToConstantHeading(new Vector2d(15, -22))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //moves the second wobble goal into box A
                .lineToLinearHeading(new Pose2d(64, 16, Math.toRadians(180)))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //pulls away from the wobble goal and out of the box
                .lineToConstantHeading(new Vector2d(40, 12))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //moves to line up with the power shot targets
                .lineToLinearHeading(new Pose2d(60.5, -27, Math.toRadians(0)))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                // moves onto the white line
                .lineToConstantHeading(new Vector2d(80, -25))
                .build());



        // shooterPosition();
        for (int i = 0; i < trajectories.size(); i++) {

            robot.drive.followTrajectory(trajectories.get(i));

            if (i == 6) {
                shooterPosition();
            }
        }

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
                .lineToConstantHeading(new Vector2d(93, -16))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //pulls back to shoot
                .lineToConstantHeading(new Vector2d(50, -16))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //moves the robot away from the rings
                .lineToConstantHeading(new Vector2d(60, -42))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //second trajectory moves the robot to line up with the second wobble goal
                .lineToLinearHeading(new Pose2d(12, -42, Math.toRadians(-90)))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //moves forward to grab the second wobble goal
                //.forward(14)
                .lineToConstantHeading(new Vector2d(5, 0))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //moves the second wobble goal into box A
                .lineToLinearHeading(new Pose2d(50, 10, Math.toRadians(180)))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //moves into box B
                .lineToConstantHeading(new Vector2d(84, -13))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //parks on white line
                .lineToConstantHeading(new Vector2d(70, -5))
                .build());






        /*
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

         */

        /*
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

         */


        for (int i = 0; i < trajectories.size(); i++) {

            if (i == 3) {
                shooterPosition();
                robot.ringShooter.compactShooter(); //robot.ringShooter.autonomousTowerShot();
            }
            robot.drive.followTrajectory(trajectories.get(i));
        }

    }

    public void positionC() {

        ArrayList<Trajectory> trajectories = new ArrayList<Trajectory>();
        trajectories.add(robot.drive.trajectoryBuilder(new Pose2d())
                //first trajectory moves the first wobble goal into box C
                //every time the robot stops, a new trajectory must be made
                .lineToConstantHeading(new Vector2d(117, 9))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //moves to line up with the tower shot
                .lineToConstantHeading(new Vector2d(51, -12))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //moves the robot away from the rings
                .lineToConstantHeading(new Vector2d(60, -40))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //second trajectory moves the robot to line up with the second wobble goal
                .lineToLinearHeading(new Pose2d(11, -40, Math.toRadians(-90))) //0
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //moves forward to grab the second wobble goal
                //.forward(14)
                .lineToConstantHeading(new Vector2d(7, -12))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //moves the second wobble goal into box A
                .lineToLinearHeading(new Pose2d(25, 11, Math.toRadians(180)))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //moves to box C
                .lineToConstantHeading(new Vector2d(104, 11))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //pulls away from the wobble goal and out of the box
                .lineToConstantHeading(new Vector2d(70, 11))
                .build());

        for (int i = 0; i < trajectories.size(); i++) {
            if (i == 2) {
                shooterPosition();
                robot.ringShooter.compactShooter();
            }
            robot.drive.followTrajectory(trajectories.get(i));
        }


    }

    public void shooterPosition() {

        robot.ringShooter.powerShot();
        robot.drive.turn(Math.toRadians(-7));
        robot.ringShooter.powerShot();
        robot.drive.turn(Math.toRadians(-7));
        robot.ringShooter.powerShot();


    }
}
