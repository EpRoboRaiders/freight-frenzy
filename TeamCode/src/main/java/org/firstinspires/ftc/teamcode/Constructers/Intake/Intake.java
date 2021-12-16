package org.firstinspires.ftc.teamcode.Constructers.Intake;

import android.graphics.RenderNode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constructers.CoreImplement;

public class Intake extends CoreImplement {

    private IntakeRoller roller = new IntakeRoller();
    private IntakeArm Flicker = new IntakeArm();


    @Override
    public void init(HardwareMap ahwMap) {
        roller.init(ahwMap);
        Flicker.init(ahwMap);
    }

    @Override
    public void update() {
        roller.update();
        Flicker.update();
    }

    public void startRoller() {
        roller.startRoller();
    }

    public void stopRoller() {
        roller.stopRoller();
    }

    public void flick() {
        Flicker.flick();
    }

    public void reverseRoller() {
        roller.reverseRoller();
    }
}
