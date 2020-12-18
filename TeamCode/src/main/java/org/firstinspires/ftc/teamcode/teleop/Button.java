package org.firstinspires.ftc.teamcode.teleop;

public class Button {

    private boolean pressed = false;
    private boolean state = false;
    public boolean checkstate (boolean button){

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

