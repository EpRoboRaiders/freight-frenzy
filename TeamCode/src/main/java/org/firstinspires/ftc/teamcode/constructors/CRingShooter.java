package org.firstinspires.ftc.teamcode.constructors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CRingShooter {

    // Motors that shoot rings in the code.
    public DcMotor leftShooter     = null;
    public DcMotor rightShooter    = null;

    // Servo that pushes rings from the hopper into the shooter mechanism.
    public Servo   shooterArm      = null;

    // Servo that raises and lowers the hopper to shoot rings.
    public Servo   hopperLifter    = null;

    private ElapsedTime shooterTimer   = new ElapsedTime();

    private final double MOTORS_OFF = 0;

    private final int HOPPER_RAISE_TIME_MS = 500; //250;
    private final int RING_SHOOT_TIME_MS   = 500;
    private final int SHOT_DELAY_TIME_MS   = 500;

    private final double SHOOTER_ARM_DISENGAGED = .5;  //.75;
    private final double SHOOTER_ARM_ENGAGED = .75; //1
    
    private final double POWERSHOT_SPEED = .63;   //.58;
    private final double TOWERSHOT_SPEED = .75;  //.69;

    //increasing this value lowers the hopper position
    private final double NO_RING_SHOT_HOPPER_DEPTH = /*0.15*/ 0.22;
    private final double TOP_RING_SHOT_HOPPER_DEPTH =  0.10; //0.09;
    private final double MIDDLE_RING_SHOT_HOPPER_DEPTH =  0.07; //0.04;
    private final double BOTTOM_RING_SHOT_HOPPER_DEPTH =  0.02;
    
    private int hopperDepth = 0;
    
    public void init(HardwareMap ahwMap) {
        leftShooter     = ahwMap.get(DcMotor.class, "left_shooter");
        rightShooter    = ahwMap.get(DcMotor.class, "right_shooter");

        shooterArm      = ahwMap.get(Servo.class, "shooter_arm");
        hopperLifter    = ahwMap.get(Servo.class, "hopper_lifter");
        
        leftShooter.setDirection(DcMotor.Direction.REVERSE);

        leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftShooter.setPower(0);
        rightShooter.setPower(0);
    }

    public void teleOpInit() {

        shooterArm.setPosition(SHOOTER_ARM_DISENGAGED);
    }

    private void hopperIncrement() {
        // Increase the box "depth" by 1, looping back to 0 after it reaches 3.
        hopperDepth += 1;
        hopperDepth = hopperDepth % 4;

        // Set the hopper box to the corresponding position.
        if (hopperDepth == 0) {
            hopperLifter.setPosition(NO_RING_SHOT_HOPPER_DEPTH);

        }
        else if (hopperDepth == 1) {
            hopperLifter.setPosition(TOP_RING_SHOT_HOPPER_DEPTH);
        }
        else if (hopperDepth == 2) {
            hopperLifter.setPosition(MIDDLE_RING_SHOT_HOPPER_DEPTH);
        }
        else { // hopperDepth == 3
            hopperLifter.setPosition(BOTTOM_RING_SHOT_HOPPER_DEPTH);

        }
        
    }
    public void ringShoot(double shotSpeed) {

        hopperIncrement();

        shooterTimer.reset();

        while (shooterTimer.milliseconds() < HOPPER_RAISE_TIME_MS) {}

        // If the hopper box has raised to a point where a ring is available to shoot,
        // do so.
        if (hopperDepth != 0) {

            setShooterPower(shotSpeed);

            if(hopperDepth == 2) {
                shooterArm.setPosition(SHOOTER_ARM_ENGAGED - .02);
            }
            else {
                shooterArm.setPosition(SHOOTER_ARM_ENGAGED);

            }


            shooterTimer.reset();

            while (shooterTimer.milliseconds() < RING_SHOOT_TIME_MS) {}

            setShooterPower(MOTORS_OFF);

            shooterArm.setPosition(SHOOTER_ARM_DISENGAGED);
        }

        // Automatically reset the hopper down to the lowest position.
        if (hopperDepth == 3) {
            hopperIncrement();
        }
        shooterTimer.reset();

        while (shooterTimer.milliseconds() < SHOT_DELAY_TIME_MS) {}

    }


    
    public void towerShot() {

        ringShoot(TOWERSHOT_SPEED);

    }
    
    public void powerShot() {

        ringShoot(POWERSHOT_SPEED);

    }

    private void setShooterPower(double power) {

        leftShooter.setPower(power);
        rightShooter.setPower(power);
    }

    public void autonomousTowerShot() {

        for (int i = 0; i < 3; i++) {
            towerShot();
        }

    }

}
