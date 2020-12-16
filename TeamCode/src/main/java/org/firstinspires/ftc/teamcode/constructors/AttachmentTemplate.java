package org.firstinspires.ftc.teamcode.constructors;

// Might be depreciated once we can figure out how to rig RobotTemplate into the RoadRunner stuff.
// Best for now to leave it like it is.

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
public class AttachmentTemplate
{

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
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public AttachmentTemplate() {

    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap           = ahwMap;

        // Define and Initialize Motors

        leftShooter     = hwMap.get(DcMotor.class, "left_shooter");
        rightShooter    = hwMap.get(DcMotor.class, "right_shooter");
        shooterArm      = hwMap.get(Servo.class, "shooter_arm");

        hopperLifter    = hwMap.get(Servo.class, "hopper_lifter");
        ringClamp       = hwMap.get(Servo.class, "ring_clamp");

        intakeArm       = hwMap.get(DcMotor.class, "intake_arm");

        wobbleGrabber   = hwMap.get(Servo.class, "wobble_grabber");
        grabberArm      = hwMap.get(Servo.class, "grabber_arm");
        clampRotator    = hwMap.get(Servo.class, "clamp_rotator");

        leftShooter.setDirection(DcMotor.Direction.REVERSE);
        // Set all motors to zero power

        leftShooter.setPower(0);
        rightShooter.setPower(0);

        intakeArm.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        intakeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set motors to brake.

        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
}