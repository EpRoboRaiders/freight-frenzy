package org.firstinspires.ftc.teamcode.constructors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.annotation.ElementType;

public class RampLoader extends CoreImplement {

    private Servo ringKicker = null;

    private ElapsedTime kickerTimer = new ElapsedTime();
    private final double KICKER_OPENED = 1; //TODO:  actual values
    private final double KICKER_CLOSED = 0; //TODO:  actual values

    private final int KICKER_KICK_MS = 200; //TODO: actual values

    private enum kickerStates {
        KICKER_RETRACTED,
        KICKER_EXTENDED;
    }

    private kickerStates kickerState = kickerStates.KICKER_RETRACTED;

    @Override
    public void update() {
        switch(kickerState){
            case KICKER_RETRACTED: //no action needed when in this state
                break;
            case KICKER_EXTENDED:
                if (kickerTimer.milliseconds() > KICKER_KICK_MS) {
                    kickerState = kickerStates.KICKER_RETRACTED;
                    ringKicker.setPosition(KICKER_OPENED);
                }
        }

    }

    @Override
    public void init(HardwareMap ahwMap) {
    }

    public void swingRingKicker() {
        kickerTimer.reset();
        kickerState = kickerStates.KICKER_EXTENDED;
        ringKicker.setPosition(KICKER_OPENED);


    }
}
