package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Cargo.CargoBoxLift;
import org.firstinspires.ftc.teamcode.Cargo.CargoBoxRotator;
import org.firstinspires.ftc.teamcode.Constructers.BaseRobot;
import org.firstinspires.ftc.teamcode.Constructers.OneShot;
@TeleOp(name= "Lift Stop Testr", group = "TeleOp")
//@Disabled
public class LiftStopTest<CargoBox> extends OpMode {


    private DcMotor boxLift;
    private  CargoBoxRotator CargoBox = new CargoBoxRotator();
    private CargoBoxLift CargoLift = new CargoBoxLift();

    OneShot powerdown = new OneShot();
    OneShot powerup = new OneShot();
    OneShot powerp0 = new OneShot();
    OneShot dumpcargo = new OneShot();
    OneShot secureCargo = new OneShot();
    OneShot bottomLevel = new OneShot();
    OneShot middleLevel = new OneShot();
    OneShot topLevel = new OneShot();

    public double liftpower = 0;


    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    @Override
    public void init() {

        double TIMEOUT_MULTIPLIERR = .2;

        // Initializes the Carousel Spinner Spinner.
        /*boxLift  = hardwareMap.get(DcMotor.class, "box_lift");//exspantion hub port 3
        boxLift.setPower(0);
        boxLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        boxLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boxLift.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        boxLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         */
        CargoBox.init(hardwareMap);
        CargoLift.init(hardwareMap);
    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */

    @Override
    public void loop() {


        if (powerup.checkState(gamepad2.dpad_up)) {
            liftpower = liftpower + 0.01;
        }
        else if (powerdown.checkState(gamepad2.dpad_down)) {
            liftpower = liftpower - 0.01;
        }
        //boxLift.setPower(liftpower);

        telemetry.addData("Lift Power", liftpower);
        //telemetry.addData("Height", boxLift.getCurrentPosition());
        telemetry.update();

        if (powerp0.checkState(gamepad2.x)) {
            liftpower = 0;
        }
        if (secureCargo.checkState(gamepad2.b)) {
            CargoBox.secureCargo();
        }
        if (dumpcargo.checkState(gamepad2.a)) {
            CargoBox.dumpCargo();
        }
        if (dumpcargo.checkState(gamepad2.y)) {
            CargoBox.resetBox();
        }
        if (bottomLevel.checkState(gamepad1.b)) {
            CargoLift.raiseCargoLift(9.4);
        }
        if (middleLevel.checkState(gamepad1.a)) {
            CargoLift.raiseCargoLift(13);
        }
        if (topLevel.checkState(gamepad1.y)) {
            CargoLift.raiseCargoLift(21.6);
        }
    }
}
