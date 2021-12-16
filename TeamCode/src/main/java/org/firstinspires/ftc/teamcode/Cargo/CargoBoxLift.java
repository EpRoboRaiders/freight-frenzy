package org.firstinspires.ftc.teamcode.Cargo;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constructers.CoreImplement;

public class CargoBoxLift extends CoreImplement {

    //adjust measurements accordingly
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 1.8125 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     LIFT_SPEED             = 0.5;
    private DcMotor boxLift;

    double TIMEOUT_MULTIPLIERR = .2;

    // Initializes the Carousel Spinner Spinner.
    @Override
    public void init(HardwareMap ahwMap) {
        boxLift  = ahwMap.get(DcMotor.class, "box_lift");//exspantion hub port 3
        boxLift.setPower(0);
        boxLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boxLift.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        boxLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    @Override
    public void update() {

    }

    public void boxMove(double power){boxLift.setPower(power * 0.65);
    }

    private void moveCargoLift(double inches, double speed){
        boxLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int target;

        // Determine new target position, and pass to motor controller
        target = boxLift.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        boxLift.setTargetPosition(target);

        //turns on run to position
        boxLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        boxLift.setPower(speed);

        ElapsedTime boxLiftTime = new ElapsedTime();

        // reset the timeout time and start motion.
        boxLiftTime.reset();

        while(
                (boxLiftTime.seconds() < Math.abs(inches) * TIMEOUT_MULTIPLIERR) &&
                        (boxLift.isBusy())){

        }

        // Turn off RUN_TO_POSITION
        boxLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boxLift.setPower(0.0);
    }

    public void raiseCargoLift(double inches) {
        moveCargoLift(inches, LIFT_SPEED);
    }

    public void lowerCargoLift(double inches) {
        moveCargoLift(inches*-1, -LIFT_SPEED);
    }
}

