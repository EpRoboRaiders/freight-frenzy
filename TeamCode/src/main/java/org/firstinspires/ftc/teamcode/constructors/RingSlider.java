package org.firstinspires.ftc.teamcode.constructors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Controls the slider for bringing rings into the box.
 * This class moves the slider that pushes the rings into the box to shoot.
 */
public class RingSlider extends CoreImplement {

    private DcMotor ringSlider = null;

    private static final double RING_SLIDER_GOING_TO_BOX_POWER = 1; //TODO:  actual values
    private static final int RING_SLIDER_GOING_TO_RAMP_POWER = -1; //TODO:  actual values

    private static final int RING_SLIDER_TO_BOX_COUNTS = 100; //TODO: actual values

    private static final int RING_SLIDER_TO_RAMP_COUNTS = 0; //TODO: actual values
    
    private boolean sliderStopped = false;

    private enum sliderStates {
        SLIDER_TO_RAMP,
        SLIDER_TO_BOX,
        IDLE;
    }

    private sliderStates sliderState = sliderStates.SLIDER_TO_RAMP;

    @Override
    public void init(HardwareMap ahwMap) {

        ringSlider   = ahwMap.get(DcMotor.class, "ring_slider");
    }

    @Override
    public void update() {
        switch(sliderState){
            case SLIDER_TO_RAMP:
                if(ringSlider.getCurrentPosition() < RING_SLIDER_TO_RAMP_COUNTS) {
                    ringSlider.setPower(MOTOR_STOP);
                    sliderState = sliderStates.IDLE;
                }
                break;
            case SLIDER_TO_BOX:
                if (ringSlider.getCurrentPosition() > RING_SLIDER_TO_BOX_COUNTS) {
                    ringSlider.setPower(RING_SLIDER_GOING_TO_RAMP_POWER);
                    sliderState = sliderStates.SLIDER_TO_RAMP;

                }
                break;
            case IDLE:
                break;
        }

    }

    public void slideRingSlider() {
        sliderState = sliderStates.SLIDER_TO_BOX;
        ringSlider.setPower(RING_SLIDER_GOING_TO_BOX_POWER);

    }
    
    public boolean finished() {
        return sliderState == sliderStates.IDLE;
    }

}