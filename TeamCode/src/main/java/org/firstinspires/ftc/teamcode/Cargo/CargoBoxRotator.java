package org.firstinspires.ftc.teamcode.Cargo;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constructers.CoreImplement;

public class CargoBoxRotator extends CoreImplement {

    private static final double DUMP_POWER = 1;
    // todo:find actualvalue
    private static final int DUMP_TIME = 1000;

    private static final double INTAKE_POSITION = 0.95;

    private static final double SECURE_POSITION = 0.7;

    private static final double DUMP_POSTION = 0.2;

    private Servo Rotator;
    @Override
    public void init(HardwareMap ahwMap) {
         Rotator = ahwMap.get(Servo.class, "rotator");//controll hub port 1
    }

    @Override
    public void update() {

    }
    public void secureCargo(){
        Rotator.setPosition(SECURE_POSITION);
    }

    public void dumpCargo() {
       Rotator.setPosition(DUMP_POSTION);

    }

    public void resetBox(){
        Rotator.setPosition(INTAKE_POSITION);
    }
}
