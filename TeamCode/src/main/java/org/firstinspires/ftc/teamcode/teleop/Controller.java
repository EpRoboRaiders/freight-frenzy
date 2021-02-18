package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constructors.Button;
import org.firstinspires.ftc.teamcode.constructors.CRingIntake;
import org.firstinspires.ftc.teamcode.constructors.OneShot;

import org.firstinspires.ftc.teamcode.constructors.TeleOpTemplate;

import static com.qualcomm.robotcore.util.Range.clip;

@TeleOp(name = "RobotController", group = "Controls")
//@Disabled
public class Controller extends OpMode {

    TeleOpTemplate robot         = new TeleOpTemplate();
    static final double SPEED         = -.5;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime looptime = new ElapsedTime();


    Button              wobbleLowered   = new Button();
    Button              wobbleUnclamped = new Button();

    OneShot             intakeStarter   = new OneShot();

    OneShot             intakeRollerToggle = new OneShot();
    boolean             intakeRollerState  = false;

    OneShot             intakeChainStarter = new OneShot();
    boolean             intakeChainStarterState = false;
    OneShot             testRampLoader     = new OneShot();
    OneShot             testRingSlider     = new OneShot();

    int                 hopperDepth   = 0;

    Button              rampLifted = new Button();

    boolean ringClamped = false;


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

    }
    @Override
    public void start() {


    }

    @Override
    public void loop() {// Cycle through the driving modes when the "b" button on the first controller is pressed.
        looptime.reset();

        // Run code depending on which drive mode is currently active (at 75% speed, because
        // our robot is too fast at full speed):
        switch(mode) {
            case TANK: {

                // In "tank" drive mode,
                // the left joystick controls the speed of the left set of motors,
                // and the right joystick -controls the right set.
                robot.drivetrain.leftFrontDrive.setPower(-gamepad1.right_stick_y * SPEED);
                robot.drivetrain.leftBackDrive.setPower(-gamepad1.right_stick_y * SPEED);
                robot.drivetrain.rightFrontDrive.setPower(-gamepad1.left_stick_y * SPEED);
                robot.drivetrain.rightBackDrive.setPower(-gamepad1.left_stick_y * SPEED);
                break;

            }
            case OMNI: {

                // Really funky. Strafes left or right using the left joystick
                // if the left joystick's "x" value is greater than "y;" runs like tank drive
                // otherwise.
                // This code was developed as a simple test by request of a coach, but the driver
                // responsible for moving the chassis actually liked the way that it worked!
                if (Math.abs(gamepad1.left_stick_x) > Math.abs(gamepad1.left_stick_y)) {
                    robot.drivetrain.leftFrontDrive.setPower(gamepad1.left_stick_x * SPEED);
                    robot.drivetrain.rightFrontDrive.setPower(-gamepad1.left_stick_x * SPEED);
                    robot.drivetrain.leftBackDrive.setPower(-gamepad1.left_stick_x * SPEED);
                    robot.drivetrain.rightBackDrive.setPower(gamepad1.left_stick_x * SPEED);
                }
                else {
                    robot.drivetrain.leftFrontDrive.setPower(-gamepad1.left_stick_y * SPEED);
                    robot.drivetrain.leftBackDrive.setPower(-gamepad1.left_stick_y * SPEED);
                    robot.drivetrain.rightFrontDrive.setPower(-gamepad1.right_stick_y * SPEED);
                    robot.drivetrain.rightBackDrive.setPower(-gamepad1.right_stick_y * SPEED);
                }
                break;

            }
            case MECANUM: {

                // Code taken from http://ftckey.com/programming/advanced-programming/. Also
                // funky; turns with the right joystick and moves/strafes with the left one.
                robot.drivetrain.leftFrontDrive.setPower(clip((-gamepad1.left_stick_y + gamepad1.left_stick_x
                        - gamepad1.right_stick_x), -1., 1) * SPEED);
                robot.drivetrain.leftBackDrive.setPower(clip((-gamepad1.left_stick_y - gamepad1.left_stick_x
                        + gamepad1.right_stick_x), -1., 1) * SPEED);
                robot.drivetrain.rightFrontDrive.setPower(clip((-gamepad1.left_stick_y - gamepad1.left_stick_x
                        - gamepad1.right_stick_x), -1., 1) * SPEED);
                robot.drivetrain.rightBackDrive.setPower(clip((-gamepad1.left_stick_y + gamepad1.left_stick_x
                        + gamepad1.right_stick_x), -1., 1) * SPEED);
                break;
            }
            case SIDESTRAFE: {
                // Strafes with the side triggers; tank drives otherwise.

                if (gamepad1.left_trigger > 0) {
                    robot.drivetrain.leftFrontDrive.setPower(-gamepad1.left_trigger * SPEED);
                    robot.drivetrain.rightFrontDrive.setPower(gamepad1.left_trigger * SPEED);
                    robot.drivetrain.leftBackDrive.setPower(gamepad1.left_trigger * SPEED);
                    robot.drivetrain.rightBackDrive.setPower(-gamepad1.left_trigger * SPEED);
                }
                else if (gamepad1.right_trigger > 0){
                    robot.drivetrain.leftFrontDrive.setPower(gamepad1.right_trigger * SPEED);
                    robot.drivetrain.rightFrontDrive.setPower(-gamepad1.right_trigger * SPEED);
                    robot.drivetrain.leftBackDrive.setPower(-gamepad1.right_trigger * SPEED);
                    robot.drivetrain.rightBackDrive.setPower(gamepad1.right_trigger * SPEED);
                }
                else {
                    robot.drivetrain.leftFrontDrive.setPower(-gamepad1.left_stick_y * SPEED);
                    robot.drivetrain.leftBackDrive.setPower(-gamepad1.left_stick_y * SPEED);
                    robot.drivetrain.rightFrontDrive.setPower(-gamepad1.right_stick_y * SPEED);
                    robot.drivetrain.rightBackDrive.setPower(-gamepad1.right_stick_y * SPEED);
                }

                break;
            }
            default: {

                mode = Mode.TANK;

                robot.drivetrain.leftFrontDrive.setPower(-gamepad1.right_stick_y * SPEED);
                robot.drivetrain.leftBackDrive.setPower(-gamepad1.right_stick_y * SPEED);
                robot.drivetrain.rightFrontDrive.setPower(-gamepad1.left_stick_y * SPEED);
                robot.drivetrain.rightBackDrive.setPower(-gamepad1.left_stick_y * SPEED);
            }
        }

        robot.ringIntake.update();

        // Set the position of the wobbleGrabber based on whetherr it is "supposed" to be clamped
        // or unclamped.

        // raises and lowers grabberArm using A button.
        robot.wobbleGrabber.raiseAndLower(wobbleLowered.checkState(gamepad2.left_stick_y > .7 || gamepad2.left_stick_y < -.7));

        // opens and closes wobbleGrabber using B button
        robot.wobbleGrabber.openAndClose(wobbleUnclamped.checkState(gamepad2.left_stick_x > .7 || gamepad2.left_stick_x < -.7));

        // Delegates the x and y buttons on Gamepad 1 to shooting rings.
        if(/*gamepad2.y ||*/ gamepad1.x) {
            // robot.ringShooter.towerShot();
            robot.ringShooter.towerShot();
        }
        else if (/*gamepad2.x ||*/ gamepad1.y) {
            robot.ringShooter.powerShot();
        }



        // Display the current mode of the robot in Telemetry for reasons deemed obvious.
        for (int i=0; i<(Mode.values().length); i++){
            telemetry.addData("Mode ", Mode.values()[i].ordinal());
            telemetry.addData("Name", Mode.values()[i]);
        }

        if (intakeRollerToggle.checkState(gamepad2.left_bumper)) {
            intakeRollerState = !intakeRollerState;
            robot.ringIntake.intakeRollerToggle(intakeRollerState);
        }

        if (intakeChainStarter.checkState(gamepad2.left_trigger >= .5)) {
            //false raises the intake roller
            intakeChainStarterState = !intakeChainStarterState;
            robot.ringIntake.intakeRollerUpDown(intakeChainStarterState);
        }

        if (testRampLoader.checkState(gamepad2.x)) {
            robot.ringIntake.testRampLoader();
        }

        if (testRingSlider.checkState(gamepad2.b)) {
            robot.ringIntake.testRingSlider();
        }

        if (intakeStarter.checkState(gamepad2.y)) {
            robot.ringIntake.ringToBox();
        }

        robot.ringIntake.raiseRampLifter(rampLifted.checkState(gamepad2.a));


        // Display other information, including the position, speed, and mode of motors.
        telemetry.addData("Robot Mode:", mode.getDescription());

        telemetry.addData("HopperDepth ", hopperDepth);

        telemetry.addData("looptime: ", looptime.milliseconds());

        telemetry.addData("armLockerPosition", robot.ringIntake.armLockerPosition());

        telemetry.update();
    }
}
