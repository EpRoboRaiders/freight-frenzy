package org.firstinspires.ftc.teamcode.Constructers;

public class NumberCruncher {



    public int incrementNumber(int number, boolean incrementorCheck) {
        if (incrementorCheck){
            number += 1;
        }
        return number;
    }

    public int decrementNumber(int number, boolean decrementorCheck) {
        if(decrementorCheck) {
            number -= 1;
        }
        return number;
    }

    public int crunchNumber(int number, boolean incrementorCheck, boolean decrementorCheck){
        if (incrementorCheck){
            number += 1;
        }
        else {
        }
        return number;
    }


}
