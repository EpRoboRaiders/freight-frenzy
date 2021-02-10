package org.firstinspires.ftc.teamcode.constructors;

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

    private static final double ROLLER_ARM_SPEED   = 1; //TODO: actual values

    private static final double ARM_LOCKER_LOWER_SPEED   = 0.5; //TODO: actual values
    private static final double ARM_LOCKER_RAISE_SPEED   = 0.5; //TODO: actual values
    private static final double ARM_LOCKER_LOWERED_COUNTS = 100; //TODO: actual values
    private static final double ARM_LOCKER_RAISED_COUNTS = 0; //TODO: actual values

    private enum rollerArmStates {
        ARM_LOWERED,
        ARM_RAISED,
        IDLE;
    }

    private rollerArmStates rollerArmState = rollerArmStates.IDLE;

    @Override
    public void init(HardwareMap ahwMap) {

        rollerArm   = ahwMap.get(DcMotor.class, "roller_arm");
        armLocker   = ahwMap.get(DcMotor.class, "arm_locker");
    }

    @Override
    public void update() {
        switch(rollerArmState) {
            case ARM_LOWERED: //no action needed when in this state
                if (armLocker.getCurrentPosition() > ARM_LOCKER_LOWERED_COUNTS) {
                    rollerArmStates rollerArmState = rollerArmStates.IDLE;
                    rollerArm.setPower(MOTOR_STOP);
                }
                break;
            case ARM_RAISED:
                if (armLocker.getCurrentPosition() < ARM_LOCKER_RAISED_COUNTS) {
                    rollerArmStates rollerArmState = rollerArmStates.IDLE;
                    rollerArm.setPower(MOTOR_STOP);
                }
                break;

            case IDLE:
                break;
        }

    }


    public void lowerRollerArm (boolean state) {

        if (state = true) {
            armLocker.setPower(ARM_LOCKER_LOWER_SPEED);
            rollerArmStates rollerArmState = rollerArmStates.ARM_LOWERED;
        }
        else {
            armLocker.setPower(ARM_LOCKER_RAISE_SPEED);
            rollerArmStates rollerArmState = rollerArmStates.ARM_RAISED;
        }

    }

    public void rollerArmActivated (boolean state) {

        if (state) {
            rollerArm.setPower(ROLLER_ARM_SPEED);
        }

        else{
            rollerArm.setPower(MOTOR_STOP);
        }
    }
}
