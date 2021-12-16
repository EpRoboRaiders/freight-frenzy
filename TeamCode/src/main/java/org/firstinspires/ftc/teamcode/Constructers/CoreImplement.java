package org.firstinspires.ftc.teamcode.Constructers;

import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class CoreImplement {
    public HardwareMap hardwareMap = null;
    protected static final double MOTOR_STOP = 0;
    public abstract void init (HardwareMap ahwMap);
    public abstract void update();

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

}
