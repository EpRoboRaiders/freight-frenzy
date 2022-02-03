package org.firstinspires.ftc.teamcode.Cargo;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constructers.CoreImplement;

public class CargoBoxLift2 extends CoreImplement {

    public static final long MILISECONDSPERINCH = 1000;
    static final double     LIFT_SPEED             = 0.5;


    private static final double DUMP_POWER = 1;
    // todo:find actualvalue
    private static final int DUMP_TIME = 1000;

    private CRServo Rotator;
    @Override
    public void init(HardwareMap ahwMap) {
        Rotator = ahwMap.get(CRServo.class, "rotator");//controll hub port 1
    }

    @Override
    public void update() {

    }
    public void boxMove(double power){Rotator.setPower(power);
    }
    private void moveCargoLift(double inches, double speed){
        Rotator.setPower(speed);
        sleep(MILISECONDSPERINCH * (long)inches);
        Rotator.setPower(0);

    }

    public void raiseCargoLift(double inches) {
        moveCargoLift(inches, LIFT_SPEED);
    }

    public void lowerCargoLift(double inches) {
        moveCargoLift(inches, -LIFT_SPEED);
    }
}
