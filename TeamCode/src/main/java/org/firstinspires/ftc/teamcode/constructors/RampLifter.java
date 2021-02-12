package org.firstinspires.ftc.teamcode.constructors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Controls ramp for transferring the ring into the box.
 * This class controls the raising and lowering of the ramp.
 */
public class RampLifter extends CoreImplement {

    private ElapsedTime rampLifterTimer = new ElapsedTime();

    private Servo rampLifter = null;

    private static final double RAMP_LIFTER_LIFTED = 0.6; //TODO:  actual values
    private static final double RAMP_LIFTER_DOWN = 1; //TODO:  actual values

    private static final int RAMP_LIFT_TIME_MS = 500;

    @Override
    public void init(HardwareMap ahwMap) {

        rampLifter   = ahwMap.get(Servo.class, "ramp_lifter");
        rampLifter.setPosition(RAMP_LIFTER_DOWN);
    }

    @Override
    public void update() {

    }

    public void setRampLifted(boolean state) {
        if (state) {
            rampLifterTimer.reset();
            rampLifter.setPosition(RAMP_LIFTER_LIFTED);
        }
        else {
            rampLifterTimer.reset();
            rampLifter.setPosition(RAMP_LIFTER_DOWN);
        }
    }

    public boolean finished() {
        return rampLifterTimer.milliseconds() > RAMP_LIFT_TIME_MS;
    }
}
