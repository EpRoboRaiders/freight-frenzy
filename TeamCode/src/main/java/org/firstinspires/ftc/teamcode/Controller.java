package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.robot.Robot;

import static com.qualcomm.robotcore.util.Range.clip;

@TeleOp(name = "RobotController", group = "Controls")
//@Disabled
public class Controller extends OpMode {
    Boolean bPressed = true;

    RobotTemplate robot = new RobotTemplate();
    // The array driveMode stores all of the possible modes for driving our robot. At the start of
    // the program, the mode is set to 0, or "tank."
    enum Mode {
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

    Mode mode = Mode.OMNI;

    public void init() {}

    @Override
    public void loop() {

    }
    private void chassisDrive() {

        // Cycle through the driving modes when the "b" button on the first controller is pressed.
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
                // and the right joystick controls the right set.
                robot.leftFrontDrive.setPower(-gamepad1.left_stick_y * .75);
                robot.leftBackDrive.setPower(-gamepad1.left_stick_y * .75);
                robot.rightFrontDrive.setPower(-gamepad1.right_stick_y * .75);
                robot.rightBackDrive.setPower(-gamepad1.right_stick_y * .75);
                break;

            }
            case OMNI: {

                // Really funky. Strafes left or right using the left joystick
                // if the left joystick's "x" value is greater than "y;" runs like tank drive
                // otherwise.
                // This code was developed as a simple test by request of a coach, but the driver
                // responsible for moving the chassis actually liked the way that it worked!
                if (Math.abs(gamepad1.left_stick_x) > Math.abs(gamepad1.left_stick_y)) {
                    robot.leftFrontDrive.setPower(gamepad1.left_stick_x * .75);
                    robot.rightFrontDrive.setPower(-gamepad1.left_stick_x * .75);
                    robot.leftBackDrive.setPower(-gamepad1.left_stick_x * .75);
                    robot.rightBackDrive.setPower(gamepad1.left_stick_x * .75);
                }
                else {
                    robot.leftFrontDrive.setPower(-gamepad1.left_stick_y * .75);
                    robot.leftBackDrive.setPower(-gamepad1.left_stick_y * .75);
                    robot.rightFrontDrive.setPower(-gamepad1.right_stick_y * .75);
                    robot.rightBackDrive.setPower(-gamepad1.right_stick_y * .75);
                }
                break;

            }
            case MECANUM: {

                // Code taken from http://ftckey.com/programming/advanced-programming/. Also
                // funky; turns with the right joystick and moves/strafes with the left one.
                robot.leftFrontDrive.setPower(clip((-gamepad1.left_stick_y + gamepad1.left_stick_x
                        + gamepad1.right_stick_x), -1., 1) * .75);
                robot.leftBackDrive.setPower(clip((-gamepad1.left_stick_y - gamepad1.left_stick_x
                        - gamepad1.right_stick_x), -1., 1) * .75);
                robot.rightFrontDrive.setPower(clip((-gamepad1.left_stick_y - gamepad1.left_stick_x
                        + gamepad1.right_stick_x), -1., 1) * .75);
                robot.rightBackDrive.setPower(clip((-gamepad1.left_stick_y + gamepad1.left_stick_x
                        - gamepad1.right_stick_x), -1., 1) * .75);
                break;
            }
            default: {

                mode = Mode.TANK;

                robot.leftFrontDrive.setPower(-gamepad1.left_stick_y * .75);
                robot.leftBackDrive.setPower(-gamepad1.left_stick_y * .75);
                robot.rightFrontDrive.setPower(-gamepad1.right_stick_y * .75);
                robot.rightBackDrive.setPower(-gamepad1.right_stick_y * .75);
            }
        }

        // Display the current mode of the robot in Telemetry for reasons deemed obvious.
        for (int i=0; i<(Mode.values().length); i++){
            telemetry.addData("Mode ", Mode.values()[i].ordinal());
            telemetry.addData("Name", Mode.values()[i]);
        }

        // Display other information, including the position, speed, and mode of motors.
        telemetry.addData("Robot Mode:", mode.getDescription());
        telemetry.addData("Motor Position:", robot.leftFrontDrive.getCurrentPosition());

        telemetry.addData("left front motor: ", robot.leftFrontDrive.getPower());
        telemetry.addData("right front motor: ", robot.rightFrontDrive.getPower());
        telemetry.addData("left back motor: ", robot.leftBackDrive.getPower());
        telemetry.addData("right back motor: ", robot.rightBackDrive.getPower());
        telemetry.update();
    }
}


