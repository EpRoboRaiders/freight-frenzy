package org.firstinspires.ftc.teamcode.constructors;

import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class CoreImplement {
    public HardwareMap hardwareMap = null;
    protected static final double MOTOR_STOP = 0;
    public abstract void init (HardwareMap ahwMap);
    public abstract void update();


}
