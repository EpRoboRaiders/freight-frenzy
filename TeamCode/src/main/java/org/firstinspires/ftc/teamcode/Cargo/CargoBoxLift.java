package org.firstinspires.ftc.teamcode.Cargo;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constructers.CoreImplement;

public class CargoBoxLift extends CoreImplement {

    //adjust measurements accordingly
    static final double     COUNTS_PER_MOTOR_REV    = 751.8 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 1.8125 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     LIFT_SPEED             = 0.7;

    private double currentInches = 0;

    private boolean liftFinished;
    private ElapsedTime boxLiftTime = new ElapsedTime();
        private DcMotor boxLift;

        static final double TIMEOUT_MULTIPLIERR = 13;

        // Initializes the Carousel Spinner Spinner.
        @Override
        public void init(HardwareMap ahwMap) {
            liftFinished = true;
            boxLift  = ahwMap.get(DcMotor.class, "box_lift");//exspantion hub port 3
            boxLift.setPower(0);
            boxLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boxLift.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        boxLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            boxLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




// 132.0344443408  9.4  13  21.6
    }

    @Override
    public void update() {
        if (!liftFinished) {
            if (boxLiftTime.seconds() > Math.abs(currentInches) * TIMEOUT_MULTIPLIERR || !boxLift.isBusy()) {

                // Turn off RUN_TO_POSITION
                boxLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                boxLift.setPower(0.0);
                liftFinished = true;
            }

        }
    }

    public void boxMove(double power){boxLift.setPower(power * 0.65);
    }

    private void moveCargoLift(double inches, double speed){

        int target;

        currentInches = inches;

        // Determine new target position, and pass to motor controller
        target = boxLift.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        boxLift.setTargetPosition(target);

        //turns on run to position
        boxLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        boxLift.setPower(speed);

        // reset the timeout time and start motion.
        boxLiftTime.reset();

        liftFinished = false;

    }

    public void raiseCargoLift(double inches) {
        moveCargoLift(inches, LIFT_SPEED);
    }

    public void lowerCargoLift(double inches) {
        moveCargoLift(inches*-1, -LIFT_SPEED);
    }

    public boolean isLiftFinished() {
        return liftFinished;
    }
}

