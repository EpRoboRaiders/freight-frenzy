package org.firstinspires.ftc.teamcode.constructors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CWobbleGrabber extends CoreImplement {

    // Servo that latches onto wobble goals.
    private Servo   wobbleGrabber    = null;

    // Servo that raises and lowers wobbleGrabber.
    private Servo   grabberArm       = null;

    private ElapsedTime clampTimer   = new ElapsedTime();

    private final double GRABBER_ARM_LOWERED = 0;
    private final double GRABBER_ARM_RAISED = 0.4;
    private final double GRABBER_ARM_HOVERED = 0.1;

    private final double WOBBLE_GRABBER_UNCLAMPED = 1;
    private final double WOBBLE_GRABBER_CLAMPED = 0.5;

    private final int    SERVO_ACTIVATION_PAUSE_MS = 500;

    private enum GrabberTransition {
        CLOSE,
        RAISE,
        OPEN,
        LOWER,
        IDLE;
    }

    private GrabberTransition grabberTransition = GrabberTransition.IDLE;

    public void init(HardwareMap ahwMap) {

        wobbleGrabber   = ahwMap.get(Servo.class, "wobble_grabber"); //TODO: actual port
        grabberArm      = ahwMap.get(Servo.class, "grabber_arm");    //TODO: actual port
    }

    public void update() {

        switch(grabberTransition) {
            case CLOSE:
                if (clampTimer.milliseconds()>= SERVO_ACTIVATION_PAUSE_MS) {
                    grabberArm.setPosition(GRABBER_ARM_RAISED);
                    clampTimer.reset();
                    grabberTransition = GrabberTransition.RAISE;

                }

            case RAISE:
                if (clampTimer.milliseconds()>= SERVO_ACTIVATION_PAUSE_MS) {
                    grabberTransition = GrabberTransition.IDLE;
                }

            case LOWER:
                if (clampTimer.milliseconds()>= SERVO_ACTIVATION_PAUSE_MS) {
                    wobbleGrabber.setPosition(WOBBLE_GRABBER_UNCLAMPED);
                    clampTimer.reset();
                    grabberTransition = GrabberTransition.OPEN;

                }

            case OPEN:
                if (clampTimer.milliseconds()>= SERVO_ACTIVATION_PAUSE_MS) {
                    grabberTransition = GrabberTransition.IDLE;
                }

            case IDLE:
                // No action needed in this state
        }

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

    public void closeAndRaiseTeleOp() {
        grabberTransition = GrabberTransition.CLOSE;
        clampTimer.reset();


        wobbleGrabber.setPosition(WOBBLE_GRABBER_CLAMPED);

    }

    public void lowerAndOpenTeleOp() {
        grabberTransition = GrabberTransition.LOWER;
        clampTimer.reset();


        grabberArm.setPosition(GRABBER_ARM_LOWERED);
    }

    public void closeAndRaise() {
        wobbleGrabber.setPosition(WOBBLE_GRABBER_CLAMPED);

        clampTimer.reset();
        while (clampTimer.milliseconds() < SERVO_ACTIVATION_PAUSE_MS) {}

        grabberArm.setPosition(GRABBER_ARM_RAISED);

        clampTimer.reset();
        while (clampTimer.milliseconds() < SERVO_ACTIVATION_PAUSE_MS) {}
    }

    public void lowerAndOpen() {
        grabberArm.setPosition(GRABBER_ARM_LOWERED);

        clampTimer.reset();
        while (clampTimer.milliseconds() < SERVO_ACTIVATION_PAUSE_MS) {}

        wobbleGrabber.setPosition(WOBBLE_GRABBER_UNCLAMPED);

        clampTimer.reset();
        while (clampTimer.milliseconds() < SERVO_ACTIVATION_PAUSE_MS) {}
    }

    public void closeAndHover() {
        wobbleGrabber.setPosition(WOBBLE_GRABBER_CLAMPED);

        clampTimer.reset();
        while (clampTimer.milliseconds() < SERVO_ACTIVATION_PAUSE_MS) {}

        grabberArm.setPosition(GRABBER_ARM_HOVERED);

        clampTimer.reset();
        while (clampTimer.milliseconds() < SERVO_ACTIVATION_PAUSE_MS) {}
    }

    public void instantLowerAndOpen() {
        grabberArm.setPosition(GRABBER_ARM_LOWERED);
        wobbleGrabber.setPosition(WOBBLE_GRABBER_UNCLAMPED);

    }
    public void instantCloseAndRaise() {
        grabberArm.setPosition(GRABBER_ARM_RAISED);
        wobbleGrabber.setPosition(WOBBLE_GRABBER_CLAMPED);

    }



}
