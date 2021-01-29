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
    private Servo   armLocker = null;

    private static final double ROLLER_ARM_RAISED  = 0; //TODO: actual values
    private static final double ROLLER_ARM_LOWERED = 1; //TODO: actual values
    private static final double ROLLER_ARM_SPEED   = 1; //TODO: actual values


    @Override
    public void init(HardwareMap ahwMap) {

        rollerArm   = ahwMap.get(DcMotor.class, "roller_arm");
        armLocker   = ahwMap.get(Servo.class, "arm_locker");
    }

    @Override
    public void update() {}


    public void lowerRollerArm (boolean state) {

        if (state) {
            armLocker.setPosition(ROLLER_ARM_LOWERED);
        }

        else {
            armLocker.setPosition(ROLLER_ARM_RAISED);
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
