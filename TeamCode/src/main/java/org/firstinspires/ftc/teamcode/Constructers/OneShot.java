package org.firstinspires.ftc.teamcode.Constructers;

import org.firstinspires.ftc.teamcode.Constructers.Button;

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