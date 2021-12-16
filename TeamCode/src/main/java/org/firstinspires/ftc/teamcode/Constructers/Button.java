package org.firstinspires.ftc.teamcode.Constructers;

/**
 * This class is used for buttons. When we want to change a button and there are multiple of them,
 * we use this to change the button and simplify how it is run.
 */
public class Button {

    protected boolean pressed = false;
    protected boolean state = false;

    /**
     * The checkstate is for checking where the object currently is. If it is at a certain position,
     * then it is either moved or it stays in that position.
     * @param button This needs to be the button that is going to be used to move the object.
     * @return
     */
    public boolean checkState (boolean button){

        // Toggle the grabberArm being raised.
        if (button && !pressed) {
            pressed = !pressed;
            state = !state;
        }
        if (!button && pressed) {
            pressed = !pressed;
        }

        return state;
    }



}


