 package org.firstinspires.ftc.teamcode.Constructers.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constructers.CoreImplement;


public class IntakeRoller extends CoreImplement {

    public DcMotor roller;

    public void init(HardwareMap ahwMap) {

        roller = ahwMap.get(DcMotor.class, "intake_roller");
    }

    @Override
    public void update() {

    }

    public void startRoller() {

        roller.setPower(-1);
    }

    public void stopRoller() {

        roller.setPower(0);
    }

    public void reverseRoller() {

        roller.setPower(1);
    }
}
