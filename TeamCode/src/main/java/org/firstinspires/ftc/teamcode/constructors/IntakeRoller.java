package org.firstinspires.ftc.teamcode.constructors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeRoller extends CoreImplement {

    private DcMotor rollerArm = null;
    private Servo   armLocker = null;

    private static final double ROLLER_ARM_RAISED  = 0; //TODO: actual values
    private static final double ROLLER_ARM_LOWERED = 1; //TODO: actual values
    private static final double ROLLER_ARM_SPEED   = 1; //TODO: actual values


    @Override
    public void init(HardwareMap ahwMap) {
        //TODO: add configuration
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
