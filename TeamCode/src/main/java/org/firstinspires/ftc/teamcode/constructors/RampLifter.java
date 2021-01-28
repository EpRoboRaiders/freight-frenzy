package org.firstinspires.ftc.teamcode.constructors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RampLifter extends CoreImplement {

    private Servo rampLifter = null;

    private static final double RAMP_LIFTER_LIFTED = 1; //TODO:  actual values
    private static final double RAMP_LIFTER_DOWN = 0; //TODO:  actual values

    @Override
    public void init(HardwareMap ahwMap) {
        //TODO: add configuration
    }

    @Override
    public void update() {

    }

    public void setRampLifted(boolean state) {
        if (state) {
            rampLifter.setPosition(RAMP_LIFTER_LIFTED);
        }
        else {
            rampLifter.setPosition(RAMP_LIFTER_DOWN);
        }
    }
}
