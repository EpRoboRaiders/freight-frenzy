package org.firstinspires.ftc.teamcode.Cargo;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constructers.CoreImplement;

public class CargoLift extends CoreImplement {

    private CargoBoxLift lift = new CargoBoxLift();

    private CargoBoxRotator boxRotator = new CargoBoxRotator();

    // Initializes the Carousel Spinner.
    @Override
    public void init(HardwareMap ahwMap) {

        boxRotator.init(ahwMap);
        lift.init(ahwMap);
    }

    @Override
    public void update() {
        boxRotator.update();
        lift.update();
    }
    public void boxMove(double power){
        lift.boxMove(power);
    }

    public void secureCargo(){
        boxRotator.secureCargo();
    }

    public void dumpCargo(){
        boxRotator.dumpCargo();
    }

    public void raiseCargoLift(double inches){
        lift.raiseCargoLift(inches);
    }

    public void lowerCargoLift(double inches){
        lift.lowerCargoLift(inches);
    }

    public void resetBox(){
        boxRotator.resetBox();
    }

    public boolean isLiftFinished() {
        return lift.isLiftFinished();
    }
}

