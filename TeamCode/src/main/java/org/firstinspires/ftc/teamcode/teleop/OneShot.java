package org.firstinspires.ftc.teamcode.teleop;

public class OneShot extends Button {

    public boolean checkState (boolean button){

        if(state) {
            state = false;
        }

        if (button && !pressed) {
            state = true;
            pressed = true;
        }

        if (!button) {
            pressed = false;
        }

        return state;
    }
}