package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.constructors.AutonomousTemplate;
import org.firstinspires.ftc.teamcode.constructors.CPipeline;
import java.util.ArrayList;


@Autonomous(name = "NewLeftBlue", group = "Autonomous")
public class NewLeftBlue extends AutonomousBase {
    private AutonomousTemplate robot = new AutonomousTemplate();

    //private final double POWER_SHOT_TURN_TO_POSITION_A =   0 ; //-10.57;
    //private final double POWER_SHOT_TURN_TO_POSITION_B =  -5 ; // -16.35 + 10.57;
    //private final double POWER_SHOT_TURN_TO_POSITION_C =  -10; //-21.80 + 5;

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


        if (robot.webcam.pipeline.getRingAmount() == CPipeline.RingPosition.NONE) {
            // Placement A
            positionA();
            // shooterPosition();
        } else if (robot.webcam.pipeline.getRingAmount() == CPipeline.RingPosition.ONE) {
            // Placement B
            positionB();
            // shooterPosition();
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
                .lineToConstantHeading(new Vector2d(73, 12))
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
                .lineToLinearHeading(new Pose2d(18, -50, Math.toRadians(-95))) //0
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //moves forward to grab the second wobble goal
                //.forward(14)
                .lineToConstantHeading(new Vector2d(18, -22))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //moves the second wobble goal into box A
                .lineToLinearHeading(new Pose2d(65, 24, Math.toRadians(180)))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //pulls away from the wobble goal and out of the box
                .lineToConstantHeading(new Vector2d(40, 15))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //moves to line up with the tower shot
                .lineToConstantHeading(new Vector2d(85, -5))
                .build());
/*
        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //spines to line up correctly
                .lineToLinearHeading(new Pose2d(-60, 8, Math.toRadians(45)))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //parks on white line
                .lineToConstantHeading(new Vector2d(69, -10))
                .build());
*/





        /*
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

         */

        /*
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

         */


        for (int i = 0; i < trajectories.size(); i++) {
            if (i == 2) {
                 shooterPosition();
            }
            robot.drive.followTrajectory(trajectories.get(i));
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
                .lineToConstantHeading(new Vector2d(95, -16))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //pulls back to shoot
                .lineToConstantHeading(new Vector2d(45, -16))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //moves the robot away from the rings
                .lineToConstantHeading(new Vector2d(60, -42))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //second trajectory moves the robot to line up with the second wobble goal
                .lineToLinearHeading(new Pose2d(1, -42, Math.toRadians(-95)))
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
                .lineToConstantHeading(new Vector2d(90, -5))
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
                 shooterPosition(); //robot.ringShooter.autonomousTowerShot();
            }
            robot.drive.followTrajectory(trajectories.get(i));
        }

    }

    public void positionC() {

        ArrayList<Trajectory> trajectories = new ArrayList<Trajectory>();
        trajectories.add(robot.drive.trajectoryBuilder(new Pose2d())
                //first trajectory moves the first wobble goal into box C
                //every time the robot stops, a new trajectory must be made
                .lineToConstantHeading(new Vector2d(120, 9))
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
                .lineToLinearHeading(new Pose2d(11, -40, Math.toRadians(-95))) //0
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //moves forward to grab the second wobble goal
                //.forward(14)
                .lineToConstantHeading(new Vector2d(11, 0))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //moves the second wobble goal into box A
                .lineToLinearHeading(new Pose2d(75, 24, Math.toRadians(180)))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //moves to box C
                .lineToConstantHeading(new Vector2d(127, 20))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //pulls away from the wobble goal and out of the box
                .lineToConstantHeading(new Vector2d(80, 20))
                .build());







        for (int i = 0; i < trajectories.size(); i++) {
            if (i == 2) {
                 shooterPosition();
            }
            robot.drive.followTrajectory(trajectories.get(i));
        }


    }

    public void shooterPosition() {

        //robot.ringIntake.extendIntake();
        robot.ringShooter.autonomousTowerShot();
        //robot.ringIntake.retractIntake();

    }
}
