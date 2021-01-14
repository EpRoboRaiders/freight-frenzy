package org.firstinspires.ftc.teamcode.constructors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CRingIntake {

    // Motor that raises and lowers the intake mechanism.
    private DcMotor intakeArm       = null;

    // Servo that clamps onto rings in the intake.
    private Servo   ringClamp       = null;

    // Servo that rotates the mechanism that clamps onto rings.
    private Servo   clampRotator    = null;

    private ElapsedTime intakeTimer   = new ElapsedTime();

    private final double RING_CLAMP_ENGAGED = .75;
    private final double RING_CLAMP_DISENGAGED = 1;

    private final double CLAMP_ROTATOR_EXTENDED = .85;
    private final double CLAMP_ROTATOR_RETRACTED = .21;

    private final int EXTEND_INTAKE_MS = 500;
    private final int RETRACT_INTAKE_MS = 800;

    private final double EXTEND_INTAKE_POWER = -.4;
    private final double RETRACT_INTAKE_POWER = .6;

    private final double SERVOS_OFF = 0;

    public enum IntakeArmPosition {
        IN_BOX,
        HOVERING,
        DOWN;
    }

    public IntakeArmPosition intakeArmPosition = IntakeArmPosition.IN_BOX;

    public IntakeArmPosition pastIntakeArmPosition = IntakeArmPosition.IN_BOX;

    public void init(HardwareMap ahwMap) {

        ringClamp       = ahwMap.get(Servo.class, "ring_clamp");
        intakeArm       = ahwMap.get(DcMotor.class, "intake_arm");
        clampRotator    = ahwMap.get(Servo.class, "clamp_rotator");

        intakeArm.setPower(0);
        intakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void clampRing() {

        ringClamp.setPosition(RING_CLAMP_ENGAGED);
    }

    public void unclampRing() {

        ringClamp.setPosition(RING_CLAMP_DISENGAGED);
    }

    public void extendClampRotator() {

        clampRotator.setPosition(CLAMP_ROTATOR_EXTENDED);
    }

    public void retractClampRotator() {

        clampRotator.setPosition(CLAMP_ROTATOR_RETRACTED);
    }

    public void controlIntakeArm(double power) {
        intakeArm.setPower(power); //-gamepad2.left_stick_y*.4
    }

    public void extendIntake() {

        intakeTimer.reset();

        intakeArm.setPower(EXTEND_INTAKE_POWER);
        while (intakeTimer.milliseconds() < EXTEND_INTAKE_MS) {}
        intakeArm.setPower(SERVOS_OFF);

    }

    public void retractIntake() {

        intakeTimer.reset();

        intakeArm.setPower(RETRACT_INTAKE_POWER);
        while (intakeTimer.milliseconds() < RETRACT_INTAKE_MS) {}
        intakeArm.setPower(SERVOS_OFF);
    }

    public int returnIntakeArmPosition() {

        return intakeArm.getCurrentPosition();
    }

    public double returnIntakeArmPower() {
        return intakeArm.getPower();
    }

    public void proportionalClampRotator() {
        int x = intakeArm.getCurrentPosition();

        if (x < -200) {
            clampRotator.setPosition(.85);
        }
        else if (x >= -200 && x <= -150) {
            clampRotator.setPosition(-0.013*x - 1.75);
        }
        else { // x > -60
            clampRotator.setPosition(.2);
        }


        // clampRotator.setPosition((0.000008513)*intakeArm.getCurrentPosition()*intakeArm.getCurrentPosition()
               //  + (0.0005874)*intakeArm.getCurrentPosition() + 0.2046);

    }

    // TODO: Actual motor instructions in each of the 6 cases where a transition is needed
    public void intakeArmPositionUpdater() {

        if (pastIntakeArmPosition == IntakeArmPosition.DOWN) {
            if (intakeArmPosition == IntakeArmPosition.HOVERING) {
                // Transition from down to hovering.
            }
            else if (intakeArmPosition == IntakeArmPosition.IN_BOX) {
                // Transition from down to in box.
            }
        }
        else if (pastIntakeArmPosition == IntakeArmPosition.HOVERING) {
            if (intakeArmPosition == IntakeArmPosition.DOWN) {
                // Transition from hovering to down.
            }
            else if (intakeArmPosition == IntakeArmPosition.IN_BOX) {
                // Transition from hovering to in box.
            }
        }
        else { /*pastIntakeArmPosition == IntakeArmPosition.IN_BOX*/
            if (intakeArmPosition == IntakeArmPosition.DOWN) {
                // Transition from in box to down.
            }
            else if (intakeArmPosition == IntakeArmPosition.HOVERING) {
                // Transition from in box to hovering.
            }
        }
    }


}
