package org.firstinspires.ftc.teamcode.constructors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RingSlider extends CoreImplement {

    private DcMotor ringSlider = null;

    private static final double RING_SLIDER_GOING_TO_BOX_POWER = 1; //TODO:  actual values
    private static final int RING_SLIDER_GOING_TO_RAMP_POWER = -1; //TODO:  actual values

    private static final int RING_SLIDER_IN_BOX_COUNTS = 100; //TODO: actual values

    private static final int RING_SLIDER_ON_RAMP_COUNTS = 0; //TODO: actual values

    private enum sliderStates {
        SLIDER_ON_RAMP,
        SLIDER_IN_BOX;
    }

    private sliderStates sliderState = sliderStates.SLIDER_ON_RAMP;

    @Override
    public void init(HardwareMap ahwMap) {
        //TODO: add configuration
    }

    @Override
    public void update() {
        switch(sliderState){
            case SLIDER_ON_RAMP:
                if(ringSlider.getCurrentPosition() < RING_SLIDER_ON_RAMP_COUNTS) {
                    ringSlider.setPower(MOTOR_STOP);
                }
                break;
            case SLIDER_IN_BOX:
                if (ringSlider.getCurrentPosition() > RING_SLIDER_IN_BOX_COUNTS) {
                    ringSlider.setPower(RING_SLIDER_GOING_TO_RAMP_POWER);
                    sliderState = sliderStates.SLIDER_ON_RAMP;

                }
                break;
        }

    }

    public void slideRingSlider() {
        sliderState = sliderStates.SLIDER_IN_BOX;
        ringSlider.setPower(RING_SLIDER_GOING_TO_BOX_POWER);

    }

}