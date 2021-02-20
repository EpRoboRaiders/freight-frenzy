package org.firstinspires.ftc.teamcode.constructors;
import org.firstinspires.ftc.teamcode.constructors.RampLoader;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends CoreImplement {

    private RampLoader rampLoader = new RampLoader();
    private RampLifter rampLifter = new RampLifter();
    private IntakeRoller intakeRoller = new IntakeRoller();
    private RingSlider ringSlider = new RingSlider();
    
    private enum IntakeTransition {
        RING_TO_RAMP,
        RAMP_LIFT,
        RING_TO_BOX,
        IDLE;
    }
    
    private IntakeTransition intakeTransition = IntakeTransition.IDLE;
    
    
    
    @Override
    public void init(HardwareMap ahwMap) {
        rampLoader.init(ahwMap);
        rampLifter.init(ahwMap);
        intakeRoller.init(ahwMap);
        ringSlider.init(ahwMap);
    }

    @Override
    public void update() {
        rampLoader.update();
        rampLifter.update();
        intakeRoller.update();
        ringSlider.update();
        
        switch (intakeTransition) {
            case RING_TO_RAMP:
                if (rampLoader.finished()) {
                    rampLifter.setRampLifted(true);
                    intakeTransition = IntakeTransition.RAMP_LIFT;
                }
                break;
            case RAMP_LIFT:
                if (rampLifter.finished()) {
                    ringSlider.slideRingSlider();
                    intakeTransition = IntakeTransition.RING_TO_BOX;
                }
                break;
            case RING_TO_BOX:
                if (ringSlider.finished()) {
                    rampLifter.setRampLifted(false);
                    intakeTransition = IntakeTransition.IDLE;
                }
                break;
            case IDLE:

                break;
        }
        

    }
    
    public void ringToBox() {
        rampLoader.swingRingKicker();
        intakeTransition = IntakeTransition.RING_TO_RAMP;

    }

    /**
     * intakeRollerToggle
     * description:
     * turns the intake roller on or off.
     * @param state tells the roller to be on or off.
     */
    public void intakeRollerToggle(boolean state) {
        intakeRoller.rollerArmActivated(state);
    }

    /**
     * intakeRollerUpDown
     * description:
     * lowers the intake arm.
     * @param state tells the roller to lower.
     */
    public void intakeRollerUpDown(boolean state) {
        intakeRoller.lowerRollerArm(state);
    }

    public void testRampLoader() {
        rampLoader.swingRingKicker();
    }

    public void testRingSlider() {
        ringSlider.slideRingSlider();
    }

    public int armLockerPosition() {
        return intakeRoller.getArmLockerEncoderCount();
    }

    public void raiseRampLifter(boolean state) {rampLifter.setRampLifted(state); }
}
