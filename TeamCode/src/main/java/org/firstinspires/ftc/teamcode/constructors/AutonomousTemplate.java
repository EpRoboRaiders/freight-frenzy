package org.firstinspires.ftc.teamcode.constructors;

// Might be depreciated once we can figure out how to rig RobotTemplate into the RoadRunner stuff.
// Best for now to leave it like it is.

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.constructors.CWobbleGrabber;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

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
public class AutonomousTemplate
{
    public CWobbleGrabber wobbleGrabber = new CWobbleGrabber();

    public CRingShooter   ringShooter   = new CRingShooter();

    public CRingIntake    ringIntake    = new CRingIntake();

    public CWebcam        webcam        = new CWebcam();

    public SampleMecanumDrive drive     = null;

    HardwareMap           hwMap         = null;

    /* Constructor */
    public AutonomousTemplate() {

    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap           = ahwMap;

        drive = new SampleMecanumDrive(hwMap);

        wobbleGrabber.init(hwMap);
        ringShooter.init(hwMap);
        ringIntake.init(hwMap);
        webcam.init(hwMap);

    }
    public void teleOpInit(HardwareMap ahwMap) {
        init(ahwMap);
        wobbleGrabber.teleOpInit();
        ringShooter.teleOpInit();

    }
}