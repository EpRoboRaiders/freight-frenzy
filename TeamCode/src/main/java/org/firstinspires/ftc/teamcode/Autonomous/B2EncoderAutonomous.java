package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constructers.BaseRobot;
import org.firstinspires.ftc.teamcode.Constructers.OneShot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "B2 Encoder Autonomous", group = "Autonomous")
public class B2EncoderAutonomous extends LinearOpMode {
    private BaseRobot robot = new BaseRobot();


    //todo: work on the scope of these objects and the delay stuf below
    private OneShot incrementDelayCheck = new OneShot();
    private OneShot decrementDelayCheck = new OneShot();
    private ElapsedTime delayTimer = new ElapsedTime();
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

        double turn1 = Math.toRadians(17);
        Trajectory myTrajectory2 = drive.trajectoryBuilder(myTrajectory1.end().plus(new Pose2d(0, 0, turn1)), false)
                .back(19.5)
                .build();

        Trajectory myTrajectory3 = drive.trajectoryBuilder(myTrajectory2.end())
                .forward(18)
                .build();

        double turn2 = Math.toRadians(70.5);
        Trajectory myTrajectory4 = drive.trajectoryBuilder(myTrajectory3.end().plus(new Pose2d(0, 0, turn2)), false)
                .forward(44)
                .build();

        double turn3 = Math.toRadians(85);
        Trajectory myTrajectory5 = drive.trajectoryBuilder(myTrajectory4.end().plus(new Pose2d(0, 0, turn3)), false)
                .forward(27)
                .build();


        waitForStart();

        robot.secureCargo();

        double liftheight = robot.getDuckPosition();

        telemetry.addData("TSE position", robot.duckpostionname());
        telemetry.update();

        drive.followTrajectory(myTrajectory1);

        drive.turn(turn1);

        drive.followTrajectory(myTrajectory2);

        robot.raiseCargoLift(liftheight);

        while (opModeIsActive() && !robot.isLiftFinished()) {
            robot.update();
        }

        sleep(100);

        robot.dumpCargo();
        sleep(1000);

        drive.followTrajectory((myTrajectory3));

        drive.turn(turn2);

        drive.followTrajectory((myTrajectory4));

        robot.startCarouselSpinner();
        sleep(3700);

        robot.stopCarouselSpinnerSpinner();

        drive.turn(turn3);

        drive.followTrajectory((myTrajectory5));

        robot.secureCargo();

        robot.lowerCargoLift(liftheight - 2);

        while (opModeIsActive() && !robot.isLiftFinished()) {
            robot.update();
        }

        /*robot.startCarouselSpinner();
        sleep(3700);
        robot.stopCarouselSpinnerSpinner();

        drive.turn(turn2);

        drive.followTrajectory((myTrajectory3));

         */

       /* Trajectory myTrajectory1 = drive.trajectoryBuilder(new Pose2d())
                .back(5)
                .build();
        double turn1 = Math.toRadians(-10);
        Trajectory myTrajectory2 = drive.trajectoryBuilder(myTrajectory1.end().plus(new Pose2d(0, 0, turn1)), false)
                .back(40)
                .build();

        double turn2 = Math.toRadians(100);
        Trajectory myTrajectory3 = drive.trajectoryBuilder(myTrajectory2.end().plus(new Pose2d(0, 0, turn2)), false)
                .back(29)
                .build();

        Trajectory myTrajectory4 = drive.trajectoryBuilder(myTrajectory3.end())
                .forward(34)
                .build();

        double turn3 = Math.toRadians(-90);
        Trajectory myTrajectory5 = drive.trajectoryBuilder(myTrajectory4.end().plus(new Pose2d(0, 0, turn3)), false)
                .forward(35.30)
                .build();

        double turn4 = Math.toRadians(-10);
        Trajectory myTrajectory6 = drive.trajectoryBuilder(myTrajectory5.end().plus(new Pose2d(0, 0, turn4)), false)
                .back(21)
                .build();

        double turn5 = Math.toRadians(15);
        Trajectory myTrajectory7 = drive.trajectoryBuilder(myTrajectory6.end().plus(new Pose2d(0, 0, turn5)), false)
                .back(21)
                .build();

        waitForStart();

        robot.secureCargo();

        drive.followTrajectory(myTrajectory1);

        drive.turn(turn1);

        drive.followTrajectory(myTrajectory2);

        drive.turn(turn2);

        drive.followTrajectory(myTrajectory3);

        robot.raiseCargoLift(21.6);
        sleep(1500);

        robot.dumpCargo();
        sleep(1000);

        drive.followTrajectory(myTrajectory4);

        drive.turn(turn3);

        drive.followTrajectory(myTrajectory5);

        robot.startCarouselSpinner();
        sleep(3700);

        robot.stopCarouselSpinnerSpinner();

        drive.turn(turn4);

        drive.followTrajectory(myTrajectory6);

        drive.turn(turn5);
*/

       /* SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.init(hardwareMap);

        // Create a delay in seconds that the robot should wait in autonomous
        int delay = 0;

        while (!gamepad1.a) {
            // If the up arrow is pressed, increase the delay by 1.
            if (incrementDelayCheck.checkState(gamepad1.dpad_up)) {
                delay += 1;
            }
            // If the down arrow is pressed, decrease the delay by 1.
            else if (decrementDelayCheck.checkState(gamepad1.dpad_down)) {
                delay -= 1;
            }
            /*
             * Clip delay to be between 0 and 30 seconds; a negative delay is impossible while
             * a delay greater than 30 seconds is longer than the autonomous period!
             */
        /*    delay = Math.min(Math.max(delay, 0), 30);
            // Display delay in autonomous.
            telemetry.addData("Start Of Autonomous Delay (press A to confirm)", delay);
            telemetry.update();
        }

        waitForStart();



        robot.secureCargo();

        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d())
                .back(42)
                .build();

        Trajectory myTrajectory2 = drive.trajectoryBuilder(myTrajectory.end().plus(new Pose2d(0, 0, Math.toRadians(90))), false)
                .back(25)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(myTrajectory);
        drive.turn(Math.toRadians(90));
        drive.followTrajectory(myTrajectory2);
        sleep(5000);*/

        /*
        //Starts under the Carousel
        robot.chassisDrive(BaseRobot.DRIVE_SPEED, -35, -35, 5);

        //Turns to line up with the Shipping Hub
        robot.chassisDrive(BaseRobot.DRIVE_SPEED, -18, 18, 5); //19 inches is about a 90 degree turn

        //Drives forward to drop off the pre-loaded box
        robot.chassisDrive(BaseRobot.DRIVE_SPEED, -26, -26, 5);

        //Drops off the pre-loaded box
        robot.raiseCargoLift(28);
        robot.dumpCargo();
        sleep(1000);
        robot.resetBox();

        //Pulls back to line up with the carousel
        robot.chassisDrive(BaseRobot.DRIVE_SPEED, 20, 20, 5);

        //Slows down before hitting wall
        robot.chassisDrive(BaseRobot.SLOW_SPEED, 13, 13, 5);

        //Turns to line up with the carousel
        robot.chassisDrive(BaseRobot.DRIVE_SPEED, -17, 17, 5);

        //Moves forward to go to the carousel
        robot.chassisDrive(BaseRobot.DRIVE_SPEED, -37.5, -37.5, 5);

        robot.startCarouselSpinner();
        sleep(3500);
        robot.stopCarouselSpinnerSpinner();

        //Lowers the lift back to the ground
        robot.lowerCargoLift(26);

        robot.chassisDrive(BaseRobot.DRIVE_SPEED,3,-3,5);

        //Pulls back into Storage Unit
        robot.chassisDrive(BaseRobot.DRIVE_SPEED, 16, 16, 5);
         */


    }
}
