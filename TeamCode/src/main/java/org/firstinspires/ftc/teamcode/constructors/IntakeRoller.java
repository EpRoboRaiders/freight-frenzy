package org.firstinspires.ftc.teamcode.constructors;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Controls the roller that sucks in rings.
 * This class raises and lowers the IntakeRoller as well as controls the speed.
 */
public class IntakeRoller extends CoreImplement {

    private DcMotor rollerArm = null;
    private DcMotor armLocker = null;
    private CRServo supplementalRoller = null;
    private CRServo speedBooster = null;

    private static final double ROLLER_ARM_SPEED   = .55; //TODO: actual values
    private static final double SUPPLEMENTAL_ROLLER_SPEED = 1; //TODO: actual values
    private static final double SPEED_BOOSTER_SPEED = 1;

    private static final double WEAK_CORRECTION_SPEED = .15;
    private static final double ARM_LOCKER_LOWER_SPEED    = -0.5;
    private static final double ARM_LOCKER_RAISE_SPEED    = 0.5; //TODO: actual values
    private static final double ARM_LOCKER_LOWERED_COUNTS = -470; //-247;
    private static final double ARM_LOCKER_RAISED_COUNTS  = 0;
    private static final double ARM_LOCKER_COUNTERACT_GRAVITY_SPEED = MOTOR_STOP;
    private static final double ARM_LOWER_SPEED_SLOPE = (ARM_LOCKER_COUNTERACT_GRAVITY_SPEED - ARM_LOCKER_LOWER_SPEED) / (ARM_LOCKER_LOWERED_COUNTS - ARM_LOCKER_RAISED_COUNTS);
    private static final double ARM_LOWER_SPEED_Y_INTERCEPT = ARM_LOCKER_LOWER_SPEED;

    private static final double ARM_LOCKER_FALL_SPEED = -.1;

    private enum armLockerStates {
        ARM_LOWERED,
        ARM_RAISED,
        DOWN_LOCK,
        IDLE;
    }

    private armLockerStates armLockerState = armLockerStates.IDLE;

    @Override
    public void init(HardwareMap ahwMap) {

        rollerArm   = ahwMap.get(DcMotor.class, "roller_arm");
        armLocker   = ahwMap.get(DcMotor.class, "arm_locker");
        supplementalRoller   = ahwMap.get(CRServo.class, "supplemental_roller");
        speedBooster = ahwMap.get(CRServo.class, "speed_boost");
        armLocker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLocker.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armLocker.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void update() {
        switch(armLockerState) {
            case ARM_LOWERED: //no action needed when in this state
                // y = mx + b
                armLocker.setPower(ARM_LOWER_SPEED_SLOPE * armLocker.getCurrentPosition() + ARM_LOWER_SPEED_Y_INTERCEPT);
                if (armLocker.getCurrentPosition() <= ARM_LOCKER_LOWERED_COUNTS) {
                    armLockerState = armLockerStates.DOWN_LOCK;
                    armLocker.setPower(ARM_LOCKER_COUNTERACT_GRAVITY_SPEED);
                }
                break;
            case ARM_RAISED:
                if (armLocker.getCurrentPosition() >= ARM_LOCKER_RAISED_COUNTS) {
                    armLockerState = armLockerStates.IDLE;
                    armLocker.setPower(MOTOR_STOP);
                }
                break;
            case DOWN_LOCK:
                if (armLocker.getCurrentPosition() > ARM_LOCKER_LOWERED_COUNTS + 1) {
                    armLocker.setPower(-WEAK_CORRECTION_SPEED); //ARM_LOWER_SPEED_SLOPE * armLocker.getCurrentPosition() + ARM_LOWER_SPEED_Y_INTERCEPT

                }
                else if (armLocker.getCurrentPosition() < ARM_LOCKER_LOWERED_COUNTS - 1) {
                    armLocker.setPower(WEAK_CORRECTION_SPEED);
                }
                else {
                    armLocker.setPower(ARM_LOCKER_COUNTERACT_GRAVITY_SPEED);
                }
                break;
            case IDLE:
                break;
        }

    }


    public void lowerRollerArm (boolean state) {

        if (state) {
            armLocker.setPower(ARM_LOCKER_LOWER_SPEED);
            armLockerState = armLockerStates.ARM_LOWERED;
        }
        else {
            armLocker.setPower(ARM_LOCKER_RAISE_SPEED);
            armLockerState = armLockerStates.ARM_RAISED;
        }

    }

    public void rollerArmActivated (boolean state) {

        if (state) {
            rollerArm.setPower(ROLLER_ARM_SPEED);
            supplementalRoller.setPower(SUPPLEMENTAL_ROLLER_SPEED);
            speedBooster.setPower(SPEED_BOOSTER_SPEED);
        }

        else{
            rollerArm.setPower(MOTOR_STOP);
            supplementalRoller.setPower(MOTOR_STOP);
            speedBooster.setPower(MOTOR_STOP);
        }
    }

    public int getArmLockerEncoderCount () {
        return armLocker.getCurrentPosition();
    }

    public String getArmLockerState() {
        switch (armLockerState) {
            case ARM_LOWERED: //no action needed when in this state
                return "Arm Lowered";
            case ARM_RAISED:
                return "Arm Raised";
            case DOWN_LOCK:
                return "Down Lock";
            case IDLE:
                return "Idle";

        }
        return "Pickle";
    }

    public void reverseSpeedBooster(boolean state) {
        if (state) {
            if (state) {
                rollerArm.setPower(-ROLLER_ARM_SPEED);
                supplementalRoller.setPower(-SUPPLEMENTAL_ROLLER_SPEED);
                speedBooster.setPower(-SPEED_BOOSTER_SPEED);
            }

            else{
                speedBooster.setPower(MOTOR_STOP);
            }
        }
    }


    public void resetArmLockerEncoder() {
        armLocker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLocker.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
