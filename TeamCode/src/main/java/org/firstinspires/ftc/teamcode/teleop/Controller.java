package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constructors.RobotTemplate;

import static com.qualcomm.robotcore.util.Range.clip;

@TeleOp(name = "RobotController", group = "Controls")
//@Disabled
public class Controller extends OpMode {

    RobotTemplate robot = new RobotTemplate();
    Boolean bPressed = true;
    // hello
    static final double SPEED = -.35;
    double grabberPower = 0;
    boolean upPressed = false;
    boolean downPressed = false;

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
    }

    @Override
    public void loop() {// Cycle through the driving modes when the "b" button on the first controller is pressed.

        if (gamepad1.b != bPressed) {

            if(!bPressed) {
                mode = mode.getNext();
            }
            bPressed = !bPressed;
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

        robot.wobbleGrabber.setPower(gamepad2.right_stick_x);

        if (gamepad2.left_trigger > 0) {
            robot.grabberArm.setPower(0.06);
        }
        else {
            robot.grabberArm.setPower(-gamepad2.left_stick_y);
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

        telemetry.addData("Wobble Grabber Servo: ", robot.wobbleGrabber.getPower());
        telemetry.addData("Grabber Arm Servo: ", robot.grabberArm.getPower());

        telemetry.update();


    }
}
