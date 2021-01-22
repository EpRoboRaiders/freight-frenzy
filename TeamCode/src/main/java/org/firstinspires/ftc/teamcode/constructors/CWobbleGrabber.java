package org.firstinspires.ftc.teamcode.constructors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CWobbleGrabber {

    // Servo that latches onto wobble goals.
    private Servo   wobbleGrabber    = null;

    // Servo that raises and lowers wobbleGrabber.
    private Servo   grabberArm       = null;

    private ElapsedTime clampTimer   = new ElapsedTime();

    private final double GRABBER_ARM_LOWERED = 0;
    private final double GRABBER_ARM_RAISED = 0.6;

    private final double WOBBLE_GRABBER_UNCLAMPED = 1;
    private final double WOBBLE_GRABBER_CLAMPED = 0.5;

    private final int    SERVO_ACTIVATION_PAUSE_MS = 500;



    public void init(HardwareMap ahwMap) {

        wobbleGrabber   = ahwMap.get(Servo.class, "wobble_grabber");
        grabberArm      = ahwMap.get(Servo.class, "grabber_arm");
    }

    public void teleOpInit() {
        wobbleGrabber.setPosition(WOBBLE_GRABBER_CLAMPED);
        grabberArm.setPosition(GRABBER_ARM_RAISED);
    }

    //raises and lowers grabberArm.
    public void raiseAndLower(boolean lowered) {
        if (lowered) {
            grabberArm.setPosition(GRABBER_ARM_LOWERED);

        }

        else {
            grabberArm.setPosition(GRABBER_ARM_RAISED);

        }
    }

    //opens and closes wobbleGrabber.
    public void openAndClose(boolean unclamped) {
        if (unclamped) {
            wobbleGrabber.setPosition(WOBBLE_GRABBER_UNCLAMPED);

        }
        else {
            wobbleGrabber.setPosition(WOBBLE_GRABBER_CLAMPED);
        }
    }


}
