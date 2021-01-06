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

    public void lowerAndUnclamp() {
        clampTimer.reset();

        grabberArm.setPosition(GRABBER_ARM_LOWERED);

        while (clampTimer.milliseconds() < SERVO_ACTIVATION_PAUSE_MS) {}

        wobbleGrabber.setPosition(WOBBLE_GRABBER_UNCLAMPED);
    }

    public void clampAndRaise() {
        clampTimer.reset();

        wobbleGrabber.setPosition(WOBBLE_GRABBER_CLAMPED);

        while (clampTimer.milliseconds() < SERVO_ACTIVATION_PAUSE_MS) {}

        grabberArm.setPosition(GRABBER_ARM_RAISED);
    }
}
