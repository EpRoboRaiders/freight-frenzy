/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.constructors;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class RobotTemplate
{
    // Motors on the drivetrain.
    public DcMotor leftFrontDrive  = null;
    public DcMotor leftBackDrive   = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive  = null;

    // Motors that shoot rings in the code.
    public DcMotor leftShooter     = null;
    public DcMotor rightShooter    = null;

    // Motor that raises and lowers the intake mechanism.
    public DcMotor intakeArm       = null;

    // Servo that latches onto wobble goals.
    public Servo   wobbleGrabber   = null;

    // Servo that raises and lowers wobbleGrabber.
    public Servo   grabberArm      = null;

    // Servo that raises and lowers the hopper to shoot rings.
    public Servo   hopperLifter    = null;

    // Servo that pushes rings from the hopper into the shooter mechanism.
    public Servo   shooterArm      = null;

    // Servo that clamps onto rings in the intake.
    public Servo   ringClamp       = null;

    // Servo that rotates the mechanism that clamps onto rings.
    public Servo   clampRotator    = null;

    /* local OpMode members. */
    HardwareMap    hwMap           = null;
    private ElapsedTime runtime  = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 5.2 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.77 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    /* Constructor */
    public RobotTemplate() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap           = ahwMap;

        // Define and Initialize Motors
        leftFrontDrive  = hwMap.get(DcMotor.class, "left_front");
        leftBackDrive   = hwMap.get(DcMotor.class, "left_back");
        rightFrontDrive = hwMap.get(DcMotor.class, "right_front");
        rightBackDrive  = hwMap.get(DcMotor.class, "right_back");

        leftShooter     = hwMap.get(DcMotor.class, "left_shooter");
        rightShooter    = hwMap.get(DcMotor.class, "right_shooter");
        shooterArm      = hwMap.get(Servo.class, "shooter_arm");

        hopperLifter    = hwMap.get(Servo.class, "hopper_lifter");
        ringClamp       = hwMap.get(Servo.class, "ring_clamp");

        intakeArm       = hwMap.get(DcMotor.class, "intake_arm");

        wobbleGrabber   = hwMap.get(Servo.class, "wobble_grabber");
        grabberArm      = hwMap.get(Servo.class, "grabber_arm");
        clampRotator    = hwMap.get(Servo.class, "clamp_rotator");



        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        leftShooter.setDirection(DcMotor.Direction.REVERSE);
        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        leftShooter.setPower(0);
        rightShooter.setPower(0);

        intakeArm.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set motors to brake.
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public void encoderDrive(double speed, double leftFrontInches, double leftBackInches,
                             double rightFrontInches, double rightBackInches, double timeoutS) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (true /* I know this is bad but don't want to break the indents below */) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            newRightBackTarget = rightBackDrive.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);

            leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            leftBackDrive.setTargetPosition(newLeftBackTarget);
            rightFrontDrive.setTargetPosition(newRightFrontTarget);
            rightBackDrive.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));

            // If rightFrontInches is positive, run the motor forward; if it is negative, run
            // backwards.9

            if (rightFrontInches > 0) {
                rightFrontDrive.setPower(Math.abs(speed));
            }
            else if(rightFrontInches < 0){
                rightFrontDrive.setPower(Math.abs(speed) * -1);
            }

            rightBackDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the  will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the  continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (/*opModeIsActive() && */
                    (runtime.seconds() < timeoutS) &&
                    (leftFrontDrive.isBusy() && leftBackDrive.isBusy()
                            /*&& .rightFrontDrive.isBusy()*/ && rightBackDrive.isBusy())) {

                // Display it for the driver.
                // telemetry.addData("Path1",  "Running to %7d :%7d %7d %7d", newLeftFrontTarget, newLeftBackTarget, newRightFrontTarget, newRightBackTarget);
                // telemetry.addData("Path2",  "Running at %7d :%7d %7d %7d",
                        //leftFrontDrive.getCurrentPosition(),
                        //leftBackDrive.getCurrentPosition(),
                        //rightFrontDrive.getCurrentPosition(),
                        //rightBackDrive.getCurrentPosition()
                //);
                // telemetry.update();
            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            runtime.reset();
            while (runtime.milliseconds() < 100) {}
            runtime.reset();
        }
    }

}
