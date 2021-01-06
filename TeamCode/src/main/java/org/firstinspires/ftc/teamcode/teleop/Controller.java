package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constructors.Button;
import org.firstinspires.ftc.teamcode.constructors.OneShot;

import org.firstinspires.ftc.teamcode.constructors.RobotTemplate;
import org.firstinspires.ftc.teamcode.constructors.TeleOpTemplate;

import static com.qualcomm.robotcore.util.Range.clip;

@TeleOp(name = "RobotController", group = "Controls")
//@Disabled
public class Controller extends OpMode {

    TeleOpTemplate robot         = new TeleOpTemplate();
    static final double SPEED         = -.5;


    double              grabberPower  = 0;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime looptime = new ElapsedTime();


    OneShot wobbleButton = new OneShot();
    boolean             armRaised = true;

    Button wobbleToggled = new Button();

    Button              ringClamped = new Button();
    Button              collectingRings = new Button();
    Button              shooterActivated = new Button();

    boolean             rotatorLocked = false;
    
    double              intakeArmPower = 0;

    int                 hopperDepth   = 0;


    // The following is COMPLETELY COSMETIC; don't touch unless you hate fun.

    //boolean moyesFound;
    //int moyesSoundID;
    //boolean leftStickPressed = false;

    // The array driveMode stores all of the possible modes for driving our robot. At the start of
    // the program, the mode is set to 0, or "tank."
    enum Mode {

        SIDESTRAFE("Strafe with Side buttons"),
        TANK("Pure Tank Drive"),
        OMNI("Hybrid Tank/Mecanum Drive"),
        MECANUM("Pure Mecanum Drive");

        private String description;

        // getNext taken from (with modifications)
        // https://digitaljoel.nerd-herders.com/2011/04/05/get-the-next-value-in-a-java-enum/.
        public Mode getNext() {
            return this.ordinal() < Mode.values().length - 1
                    ? Mode.values()[this.ordinal() + 1]
                    : Mode.values()[0];
        }

        // Code taken from (with modifications)
        // https://stackoverflow.com/questions/15989316/
        // how-to-add-a-description-for-each-entry-of-enum/15989359#15989359
        private Mode(String description) {
            this.description = description;
        }

        public String getDescription() {
            return description;
        }
    }

    Mode mode = Mode.SIDESTRAFE;

    public void init() {

        robot.teleOpInit(hardwareMap);

        /*
        // Also cosmetic
        int moyesSoundID = hardwareMap.appContext.getResources().getIdentifier("moyes", "raw", hardwareMap.appContext.getPackageName());

        if (moyesSoundID != 0)
            moyesFound   = SoundPlayer.getInstance().preload(hardwareMap.appContext, moyesSoundID);

         */

    }

    @Override
    public void loop() {// Cycle through the driving modes when the "b" button on the first controller is pressed.
        looptime.reset();
        /*
        if (gamepad1.y != y1Pressed) {

            if(!y1Pressed) {
                mode = mode.getNext();
            }
            y1Pressed = !y2Pressed;
        }

         */



        // Run code depending on which drive mode is currently active (at 75% speed, because
        // our robot is too fast at full speed):
        switch(mode) {
            case TANK: {

                // In "tank" drive mode,
                // the left joystick controls the speed of the left set of motors,
                // and the right joystick -controls the right set.
                robot.leftFrontDrive.setPower(-gamepad1.right_stick_y * SPEED);
                robot.leftBackDrive.setPower(-gamepad1.right_stick_y * SPEED);
                robot.rightFrontDrive.setPower(-gamepad1.left_stick_y * SPEED);
                robot.rightBackDrive.setPower(-gamepad1.left_stick_y * SPEED);
                break;

            }
            case OMNI: {

                // Really funky. Strafes left or right using the left joystick
                // if the left joystick's "x" value is greater than "y;" runs like tank drive
                // otherwise.
                // This code was developed as a simple test by request of a coach, but the driver
                // responsible for moving the chassis actually liked the way that it worked!
                if (Math.abs(gamepad1.left_stick_x) > Math.abs(gamepad1.left_stick_y)) {
                    robot.leftFrontDrive.setPower(gamepad1.left_stick_x * SPEED);
                    robot.rightFrontDrive.setPower(-gamepad1.left_stick_x * SPEED);
                    robot.leftBackDrive.setPower(-gamepad1.left_stick_x * SPEED);
                    robot.rightBackDrive.setPower(gamepad1.left_stick_x * SPEED);
                }
                else {
                    robot.leftFrontDrive.setPower(-gamepad1.left_stick_y * SPEED);
                    robot.leftBackDrive.setPower(-gamepad1.left_stick_y * SPEED);
                    robot.rightFrontDrive.setPower(-gamepad1.right_stick_y * SPEED);
                    robot.rightBackDrive.setPower(-gamepad1.right_stick_y * SPEED);
                }
                break;

            }
            case MECANUM: {

                // Code taken from http://ftckey.com/programming/advanced-programming/. Also
                // funky; turns with the right joystick and moves/strafes with the left one.
                robot.leftFrontDrive.setPower(clip((-gamepad1.left_stick_y + gamepad1.left_stick_x
                        - gamepad1.right_stick_x), -1., 1) * SPEED);
                robot.leftBackDrive.setPower(clip((-gamepad1.left_stick_y - gamepad1.left_stick_x
                        + gamepad1.right_stick_x), -1., 1) * SPEED);
                robot.rightFrontDrive.setPower(clip((-gamepad1.left_stick_y - gamepad1.left_stick_x
                        - gamepad1.right_stick_x), -1., 1) * SPEED);
                robot.rightBackDrive.setPower(clip((-gamepad1.left_stick_y + gamepad1.left_stick_x
                        + gamepad1.right_stick_x), -1., 1) * SPEED);
                break;
            }
            case SIDESTRAFE: {
                // Strafes with the side triggers; tank drives otherwise.

                if (gamepad1.left_trigger > 0) {
                    robot.leftFrontDrive.setPower(-gamepad1.left_trigger * SPEED);
                    robot.rightFrontDrive.setPower(gamepad1.left_trigger * SPEED);
                    robot.leftBackDrive.setPower(gamepad1.left_trigger * SPEED);
                    robot.rightBackDrive.setPower(-gamepad1.left_trigger * SPEED);
                }
                else if (gamepad1.right_trigger > 0){
                    robot.leftFrontDrive.setPower(gamepad1.right_trigger * SPEED);
                    robot.rightFrontDrive.setPower(-gamepad1.right_trigger * SPEED);
                    robot.leftBackDrive.setPower(-gamepad1.right_trigger * SPEED);
                    robot.rightBackDrive.setPower(gamepad1.right_trigger * SPEED);
                }
                else {
                    robot.leftFrontDrive.setPower(-gamepad1.right_stick_y * SPEED);
                    robot.leftBackDrive.setPower(-gamepad1.right_stick_y * SPEED);
                    robot.rightFrontDrive.setPower(-gamepad1.left_stick_y * SPEED);
                    robot.rightBackDrive.setPower(-gamepad1.left_stick_y * SPEED);
                }

                break;
            }
            default: {

                mode = Mode.TANK;

                robot.leftFrontDrive.setPower(-gamepad1.right_stick_y * SPEED);
                robot.leftBackDrive.setPower(-gamepad1.right_stick_y * SPEED);
                robot.rightFrontDrive.setPower(-gamepad1.left_stick_y * SPEED);
                robot.rightBackDrive.setPower(-gamepad1.left_stick_y * SPEED);
            }
        }

        // Set the position of the wobbleGrabber based on whether it is "supposed" to be clamped
        // or unclamped.

        if (wobbleButton.checkState(gamepad2.b)) {

            armRaised = !armRaised;
            if (armRaised) {
                robot.wobbleGrabber.clampAndRaise();

            }
            else {
                robot.wobbleGrabber.lowerAndUnclamp();
            }
        }

        if(gamepad2.y) {
            // robot.ringShooter.towerShot();
            robot.ringShooter.ringShoot(.7, .5);
        }
        else if (gamepad2.x) {
            robot.ringShooter.powerShot();
        }


        // Set the ringClamp to a corresponding state based on if ringClamped is true.
        if (ringClamped.checkState(gamepad2.right_bumper)) {
            robot.ringIntake.clampRing();
        }
        else {
            robot.ringIntake.clampRing();
        }

        // Set the clampRotator to a corresponding state based on if collectingRings is true.
        if (collectingRings.checkState(gamepad2.right_trigger >.2)) {
            robot.ringIntake.extendClampRotator();
        }
        else {
            robot.ringIntake.retractClampRotator();
        }

        robot.ringIntake.controlIntakeArm(-gamepad2.left_stick_y*.4);



        // Map the clampRotator to the x axis of the second gamepad's right joystick.
        // robot.clampRotator.setPosition(gamepad2.right_stick_x);

        // robot.ringClamp.setPosition(gamepad2.right_stick_x);

        //robot.wobbleGrabber.setPosition(gamepad2.right_stick_x);
        //gamepad2.left_stick_x

        // If the X button is pressed, activate the shooter arm and the shooter mechanism itself.


        /*
        if(gamepad2.x) {
            robot.leftShooter.setPower(.71);
            robot.rightShooter.setPower(.71);
            robot.shooterArm.setPosition(1);
        }
        // Less powerful shot for the Power Targets.
        else if(gamepad2.left_trigger > .2) {
            robot.leftShooter.setPower(.61);
            robot.rightShooter.setPower(.61);
            robot.shooterArm.setPosition(1);
        }
        else {
            robot.leftShooter.setPower(0);
            robot.rightShooter.setPower(0);
            robot.shooterArm.setPosition(.8);
        }

         */





        //robot.wobbleGrabber.setPosition(gamepad2.right_stick_x);
        /*
        if (gamepad2.left_trigger > 0) {
            robot.grabberArm.setPower(0.06);
        }
        else {
            robot.grabberArm.setPower(-gamepad2.left_stick_y);
        }
         */

        /*
        // Also cosmetic
        if (gamepad2.left_stick_button && !leftStickPressed) {
            leftStickPressed = !leftStickPressed;
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, moyesSoundID);

        }
        if (!gamepad2.left_stick_button && leftStickPressed) {
            leftStickPressed = !leftStickPressed;
        }

         */

        // Display the current mode of the robot in Telemetry for reasons deemed obvious.
        for (int i=0; i<(Mode.values().length); i++){
            telemetry.addData("Mode ", Mode.values()[i].ordinal());
            telemetry.addData("Name", Mode.values()[i]);
        }

        // Display other information, including the position, speed, and mode of motors.
        telemetry.addData("Robot Mode:", mode.getDescription());
        // telemetry.addData("Motor Position:", robot.leftFrontDrive.getCurrentPosition());

        telemetry.addData("Left Front Motor: ", robot.leftFrontDrive.getPower());
        telemetry.addData("Right Front Motor: ", robot.rightFrontDrive.getPower());
        telemetry.addData("Left Back Motor: ", robot.leftBackDrive.getPower());
        telemetry.addData("Fight Back Motor: ", robot.rightBackDrive.getPower());

        // telemetry.addData("Wobble Grabber Servo: ", robot.wobbleGrabber.getPosition());
        // telemetry.addData("Grabber Arm Servo: ", robot.grabberArm.getPosition());
        telemetry.addData("HopperDepth ", hopperDepth);

        // telemetry.addData("Ring Clamp: ", robot.ringClamp.getPosition());
        // telemetry.addData("Clamp Rotator: ", robot.clampRotator.getPosition());
        // telemetry.addData("intakeArm :", robot.intakeArm.getCurrentPosition());

        telemetry.addData("looptime :", looptime.milliseconds());
        // telemetry.addData("gold resource",   moyesFound ?   "Found" : "NOT found\n Add moyes.wav to /src/main/res/raw" );

        telemetry.update();


    }
}
