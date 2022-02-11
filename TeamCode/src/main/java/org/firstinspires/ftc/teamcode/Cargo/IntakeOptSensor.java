package org.firstinspires.ftc.teamcode.Cargo;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constructers.CoreImplement;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import java.util.Locale;


public class IntakeOptSensor extends CoreImplement {
    DistanceSensor sensorDistance;
    @Override
    public void init(HardwareMap ahwMap) {
        sensorDistance = ahwMap.get(DistanceSensor.class, "box_sensor");

    }

    @Override
    public void update() {

    }

    public boolean ifsensed() {
       return sensorDistance.getDistance(DistanceUnit.CM) < 3;
    }

}
