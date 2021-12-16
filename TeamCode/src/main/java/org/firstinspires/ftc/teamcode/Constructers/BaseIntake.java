package org.firstinspires.ftc.teamcode.Constructers;
/**
 * This class will turn on and off hte intake
 */
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BaseIntake extends CoreImplement {
    @Override
    public void init(HardwareMap ahwMap){}
    /**
     * this sets the state whether the intake is one or off
     */
    private boolean state = false;
    /**
     * this will turn on the intake
     */
    public void turnOnIntake() {
        state = true;
    }
    /**
     *this will turn on the intake
     */
    public void turnOffIntake() {
        state = false;
    }

    /**
     * this will make the motors turn on and off  automaticly
     */
    @Override
    public void update() {}


}
