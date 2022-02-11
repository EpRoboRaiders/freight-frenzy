package org.firstinspires.ftc.teamcode.Cargo;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constructers.CoreImplement;
import org.firstinspires.ftc.teamcode.Constructers.Intake.Intake;

public class CargoBoxRotator extends CoreImplement {

    private static final double DUMP_POWER = 1;
    // todo:find actualvalue
    private static final int DUMP_TIME = 1000;

    private static final double INTAKE_POSITION = 0.77;

    private static final double SECURE_POSITION = 0.42;

    private static final double DUMP_POSTION = 0.05;

    private IntakeSensor touchSensor = new IntakeSensor();

    private IntakeOptSensor DistanceSensor = new IntakeOptSensor();


    private boolean readyToIntake = true;


    private Servo Rotator;

    @Override
    public void init(HardwareMap ahwMap) {
        Rotator = ahwMap.get(Servo.class, "rotator");//controll hub port 5
        DistanceSensor.init(ahwMap);
    }


    @Override
    public void update() {
        if (readyToIntake && ispressed()) {
            secureCargo();
        }
    }

    public void secureCargo() {
        Rotator.setPosition(SECURE_POSITION);
        readyToIntake = false;
    }

    public void dumpCargo() {
        Rotator.setPosition(DUMP_POSTION);
        readyToIntake = false;
    }

    public void resetBox() {
        Rotator.setPosition(INTAKE_POSITION);
        readyToIntake = true;
    }

    public void positionset(double position) {
        Rotator.setPosition(position);
    }

    private boolean ispressed() {
        return         DistanceSensor.ifsensed();

    }


}
