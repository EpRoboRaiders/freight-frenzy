package org.firstinspires.ftc.teamcode.Cargo;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Constructers.CoreImplement;

public class IntakeSensor extends CoreImplement {

    TouchSensor TouchSensor;  // Hardware Device Object

    @Override
    public void init(HardwareMap ahwMap) {
        TouchSensor = ahwMap.get(TouchSensor.class, "touch_sensor");
    }

    @Override
    public void update() {

    }

    public boolean ispressed() {
        return TouchSensor.isPressed();
    }
}

