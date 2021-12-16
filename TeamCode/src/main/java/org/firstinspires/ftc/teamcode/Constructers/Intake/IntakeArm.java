package org.firstinspires.ftc.teamcode.Constructers.Intake;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constructers.CoreImplement;

public class IntakeArm extends CoreImplement {

    private Servo Flicker;
    private ElapsedTime retractiontimer = new ElapsedTime();

    @Override
    public void init(HardwareMap ahwMap) {
        Flicker = ahwMap.get(Servo.class, "flicker");
    }

    @Override
    public void update() {

        if (retractiontimer.seconds() > 1) {
            Flicker.setPosition(1);
        }
    }


    public void flick() {
        Flicker.setPosition(.7);
        retractiontimer.reset();
    }
}
