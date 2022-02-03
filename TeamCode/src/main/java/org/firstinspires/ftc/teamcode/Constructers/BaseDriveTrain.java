package org.firstinspires.ftc.teamcode.Constructers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BaseDriveTrain extends CoreImplement {

    HardwareMap hwMap           =  null;

    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    // powerSwitch switches the power for the motors that way we can change the front and the back.
    private int powerSwitch;

    private ElapsedTime driveTime = new ElapsedTime();
// makes the left side faster to compensate for drift
    private static double LEFT_MULTIPLIER = 36.5/33;


    //adjust measurements accordingly
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 5.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;
    static final double     SLOW_SPEED              = 0.2;
    static final double     TURN_SPEED              = 0.5;

    public BaseDriveTrain(){

    }

    @Override
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFrontDrive  = hwMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hwMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hwMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hwMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors

        powerSwitch = 1;

        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void update() {

    }



    public void setLeftBackMotorPower(double power){
        leftBackDrive.setPower(power * powerSwitch);
    }
    public void setLeftFrontMotorPower(double power){leftFrontDrive.setPower(power * powerSwitch);}
    public void setRightBackMotorPower(double power){rightBackDrive.setPower(power * powerSwitch);}
    public void setRightFrontMotorPower(double power){rightFrontDrive.setPower(power * powerSwitch);}

    public void setChassisLeftSidePower(double power){
        this.setLeftFrontMotorPower(power);
        this.setLeftBackMotorPower(power);
    }

    public void setChassisRightSidePower(double power){
        this.setRightFrontMotorPower(power);
        this.setRightBackMotorPower(power);
    }

    public void setChassisTankDrivePower(double leftPower, double rightPower){

//this if else statement switch the side that the joysticks controll for the chassis
        if (powerSwitch == -1){
            this.setChassisLeftSidePower(rightPower);
            this.setChassisRightSidePower(leftPower);
        }

        else {
            this.setChassisLeftSidePower(leftPower);
            this.setChassisRightSidePower(rightPower);
        }
    }
    public void chassisDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
// sets the encoders to zero
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// defines the left and right target
        int leftTarget;
        int rightTarget;

        // Determine new target position, and pass to motor controller
        leftTarget = leftFrontDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH * LEFT_MULTIPLIER);
        rightTarget = rightFrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
        leftFrontDrive.setTargetPosition(leftTarget);
        rightFrontDrive.setTargetPosition(rightTarget);
        leftBackDrive.setTargetPosition(leftTarget);
        rightBackDrive.setTargetPosition(rightTarget);

        //turns on run to position
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setChassisTankDrivePower(Math.abs(speed),Math.abs(speed));

        // reset the timeout time and start motion.
        driveTime.reset();

        while(
                (driveTime.seconds() < timeoutS) &&
                (leftFrontDrive.isBusy() && rightFrontDrive.isBusy())) {

        }

        // Turn off RUN_TO_POSITION
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setChassisTankDrivePower(0.0,0.0);

    }
// the power switch changes the direction for the motors
    public void directionSwitch(){
        powerSwitch = powerSwitch * -1;
    }

    public String debug() {
        return (powerSwitch == 1)? "forward":"backwards";
    }
}