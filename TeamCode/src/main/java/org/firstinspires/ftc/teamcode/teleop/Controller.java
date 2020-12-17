package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constructors.RobotTemplate;

import static com.qualcomm.robotcore.util.Range.clip;

@TeleOp(name = "RobotController", group = "Controls")
//@Disabled
public class Controller extends OpMode {

    RobotTemplate       robot         = new RobotTemplate();
    static final double SPEED         = -.5;
    double              grabberPower  = 0;

    boolean             upPressed     = false;
    boolean             downPressed   = false;
    boolean             aPressed      = false;
    boolean             bPressed      = false;
    boolean             y2Pressed     = false;
    boolean             y1Pressed     = false;
    boolean             rightBumperPressed = false;
    boolean             leftBumperPressed = false;
    boolean             rightTriggerPressed = false;

    boolean             wobbleClamped = true;
    boolean             armRaised     = true;
    boolean             ringClamped   = false;
    boolean             collectingRings = true;
    boolean             rotatorLocked = false;
    
    double              intakeArmPower = 0;

    int                 hopperDepth   = 0;

    // The following is COMPLETELY COSMETIC; don't touch unless you hate fun.

    boolean moyesFound;
    int moyesSoundID;
    boolean leftStickPressed = false;

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
        robot.init(hardwareMap);

        // Also cosmetic
        int moyesSoundID = hardwareMap.appContext.getResources().getIdentifier("moyes", "raw", hardwareMap.appContext.getPackageName());

        if (moyesSoundID != 0)
            moyesFound   = SoundPlayer.getInstance().preload(hardwareMap.appContext, moyesSoundID);

    }

    @Override
    public void loop() {// Cycle through the driving modes when the "b" button on the first controller is pressed.

        if (gamepad1.b != y1Pressed) {

            if(!y1Pressed) {
                mode = mode.getNext();
            }
            y1Pressed = !y2Pressed;
        }

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

        // Toggle the grabberArm being raised.
        if (gamepad2.b && !bPressed) {
            bPressed = !bPressed;
            wobbleClamped = !wobbleClamped;
        }
        if (!gamepad2.b && bPressed) {
            bPressed = !bPressed;
        }

        // Toggle the wobbleGrabber being clamped.
        if (gamepad2.a && !aPressed) {
            aPressed = !aPressed;
            armRaised = !armRaised;
        }
        if (!gamepad2.a && aPressed) {
            aPressed = !aPressed;
        }

        // Set the position of the wobbleGrabber based on whether it is "supposed" to be clamped
        // or unclamped.
        if (wobbleClamped) {
            robot.wobbleGrabber.setPosition(1);
        }
        else {
            robot.wobbleGrabber.setPosition(0.5);
        }

        // Set the position of the grabberArm based on whether it is "supposed" to be up or down.

        if (armRaised) {
            robot.grabberArm.setPosition(0.6);
        }
        else {
            robot.grabberArm.setPosition(0);
        }
        /*
        if (gamepad2.y && !y2Pressed) {
            y2Pressed = !y2Pressed;
            hopperDepth += 1;
        }
        if (!gamepad2.y && y2Pressed) {
            y2Pressed = !y2Pressed;
        }

        hopperDepth = hopperDepth % 4;
         */

        // Cycle hopperDepth between 0 and 3, incrementing by 1 when the y button is pressed
        // (and back to 0 when it exceeds 3).
        if (gamepad2.y && !y2Pressed) {
            y2Pressed = !y2Pressed;
            hopperDepth += 1;
        }
        if (!gamepad2.y && y2Pressed) {
            y2Pressed = !y2Pressed;
        }
        hopperDepth = hopperDepth % 4;


        // Set the hopperLifter to a corresponding position based on the hopperDepth.
        if (hopperDepth == 0) {
            robot.hopperLifter.setPosition(0.15);
        }
        else if (hopperDepth == 1) {
            robot.hopperLifter.setPosition(0.09);
        }
        else if (hopperDepth == 2) {
            robot.hopperLifter.setPosition(0.03);
        }
        else {
            robot.hopperLifter.setPosition(0);
        }

        // Toggle ringClamped using the right bumper on the second controller.
        if (gamepad2.right_bumper && !rightBumperPressed) {
            rightBumperPressed = !rightBumperPressed;
            ringClamped = !ringClamped;
        }
        if (!gamepad2.right_bumper && rightBumperPressed) {
            rightBumperPressed = !rightBumperPressed;
        }

        // Set the ringClamp to a corresponding state based on if ringClamped is true.
        if (ringClamped) {
            robot.ringClamp.setPosition(.75);
        }
        else {
            robot.ringClamp.setPosition(1);
        }

        // Map the clampRotator to the x axis of the second gamepad's right joystick.
        // robot.clampRotator.setPosition(gamepad2.right_stick_x);





        // robot.ringClamp.setPosition(gamepad2.right_stick_x);

        //robot.wobbleGrabber.setPosition(gamepad2.right_stick_x);
        //gamepad2.left_stick_x

        // If the X button is pressed, activate the shooter arm and the shooter mechanism itself.

        if(gamepad2.x) {
            robot.leftShooter.setPower(.71);
            robot.rightShooter.setPower(.71);
            robot.shooterArm.setPosition(1);
        }
        else if(gamepad2.left_trigger > .2) {
            robot.leftShooter.setPower(.62);
            robot.rightShooter.setPower(.62);
            robot.shooterArm.setPosition(1);
        }
        else {
            robot.leftShooter.setPower(0);
            robot.rightShooter.setPower(0);
            robot.shooterArm.setPosition(.8);
        }

        // Toggle collectingRings using the right trigger on the second controller.
        if (gamepad2.right_trigger > .2 && !rightTriggerPressed) {
            rightTriggerPressed = !rightTriggerPressed;
            collectingRings = !collectingRings;
        }
        if (gamepad2.right_trigger <= .2 && rightTriggerPressed) {
            rightTriggerPressed = !rightTriggerPressed;
        }

        // Set the clampRotator to a corresponding state based on if collectingRings is true.
        if (collectingRings) {
            robot.clampRotator.setPosition(.85);
        }
        else {
            robot.clampRotator.setPosition(.21);
        }

        if (gamepad2.left_bumper && !leftBumperPressed) {
            leftBumperPressed = !leftBumperPressed;
            rotatorLocked = !rotatorLocked;
            intakeArmPower = robot.intakeArm.getPower();
            
        }
        if (!gamepad2.left_bumper && leftBumperPressed) {
            leftBumperPressed = !leftBumperPressed;
        }
        
        if (rotatorLocked) {
            robot.intakeArm.setPower(intakeArmPower);
        }
        else {
            // Raise or lower the intake arm with the left stick.
            robot.intakeArm.setPower(-gamepad2.left_stick_y*.4);
        }

        //robot.wobbleGrabber.setPosition(gamepad2.right_stick_x);
        /*
        if (gamepad2.left_trigger > 0) {
            robot.grabberArm.setPower(0.06);
        }
        else {
            robot.grabberArm.setPower(-gamepad2.left_stick_y);
        }
         */

        // Also cosmetic
        if (gamepad2.left_stick_button && !leftStickPressed) {
            leftStickPressed = !leftStickPressed;
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, moyesSoundID);

        }
        if (!gamepad2.left_stick_button && leftStickPressed) {
            leftStickPressed = !leftStickPressed;
        }

        // Display the current mode of the robot in Telemetry for reasons deemed obvious.
        for (int i=0; i<(Mode.values().length); i++){
            telemetry.addData("Mode ", Mode.values()[i].ordinal());
            telemetry.addData("Name", Mode.values()[i]);
        }

        // Display other information, including the position, speed, and mode of motors.
        telemetry.addData("Robot Mode:", mode.getDescription());
        telemetry.addData("Motor Position:", robot.leftFrontDrive.getCurrentPosition());

        telemetry.addData("Left Front Motor: ", robot.leftFrontDrive.getPower());
        telemetry.addData("Right Front Motor: ", robot.rightFrontDrive.getPower());
        telemetry.addData("Left Back Motor: ", robot.leftBackDrive.getPower());
        telemetry.addData("Fight Back Motor: ", robot.rightBackDrive.getPower());

        telemetry.addData("Wobble Grabber Servo: ", robot.wobbleGrabber.getPosition());
        telemetry.addData("Grabber Arm Servo: ", robot.grabberArm.getPosition());
        telemetry.addData("HopperDepth ", hopperDepth);

        telemetry.addData("Ring Clamp: ", robot.ringClamp.getPosition());
        telemetry.addData("Clamp Rotator: ", robot.clampRotator.getPosition());
        telemetry.addData("intakeArm :", robot.intakeArm.getCurrentPosition());
        telemetry.addData("gold resource",   moyesFound ?   "Found" : "NOT found\n Add moyes.wav to /src/main/res/raw" );

        telemetry.update();


    }
}
