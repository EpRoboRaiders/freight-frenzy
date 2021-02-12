package org.firstinspires.ftc.teamcode.constructors;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Controls the slider for bringing rings into the box.
 * This class moves the slider that pushes the rings into the box to shoot.
 */
public class RingSlider extends CoreImplement {

    private CRServo ringSlider = null;

    private static final double RING_SLIDER_GOING_TO_BOX_POWER = 1; //TODO:  actual values
    private static final int RING_SLIDER_GOING_TO_RAMP_POWER = -1; //TODO:  actual values

    private static final int RING_SLIDER_TO_BOX_MS = 650;

    private static final int RING_SLIDER_TO_RAMP_MS = 585;
    
    private boolean sliderStopped = false;

    private enum sliderStates {
        SLIDER_TO_RAMP,
        SLIDER_TO_BOX,
        IDLE;
    }

    private ElapsedTime sliderTimer = new ElapsedTime();

    private sliderStates sliderState = sliderStates.SLIDER_TO_RAMP;

    @Override
    public void init(HardwareMap ahwMap) {

        ringSlider   = ahwMap.get(CRServo.class, "ring_slider");
    }

    @Override
    public void update() {
        switch(sliderState){
            case SLIDER_TO_RAMP:
                if(sliderTimer.milliseconds() > RING_SLIDER_TO_RAMP_MS) {
                    ringSlider.setPower(MOTOR_STOP);
                    sliderState = sliderStates.IDLE;
                    sliderTimer.reset();
                }
                break;
            case SLIDER_TO_BOX:
                if (sliderTimer.milliseconds() > RING_SLIDER_TO_BOX_MS) {
                    ringSlider.setPower(RING_SLIDER_GOING_TO_RAMP_POWER);
                    sliderState = sliderStates.SLIDER_TO_RAMP;
                    sliderTimer.reset();

                }
                break;
            case IDLE:
                break;
        }

    }

    public void slideRingSlider() {
        sliderState = sliderStates.SLIDER_TO_BOX;
        sliderTimer.reset();
        ringSlider.setPower(RING_SLIDER_GOING_TO_BOX_POWER);

    }
    
    public boolean finished() {
        return sliderState == sliderStates.IDLE;
    }

}