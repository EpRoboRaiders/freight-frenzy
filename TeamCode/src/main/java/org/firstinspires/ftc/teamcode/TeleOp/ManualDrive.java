package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Cargo.CargoLift;
import org.firstinspires.ftc.teamcode.Constructers.BaseDriveTrain;
import org.firstinspires.ftc.teamcode.Constructers.BaseRobot;
import org.firstinspires.ftc.teamcode.Constructers.Button;
import org.firstinspires.ftc.teamcode.Constructers.OneShot;

@TeleOp (name= "Manual Drive", group = "TeleOp")
//@Disabled
public class ManualDrive extends OpMode
{



    BaseRobot robot = new BaseRobot();

    Button IntakeRoller = new Button();
    Button IntakeReverseRoller = new Button();

    //OneShot IntakeArm = new OneShot();

    OneShot dumpCargoCheck = new OneShot();
    OneShot secureCargoCheck = new OneShot();
    OneShot resetBox = new OneShot();
    OneShot shortlift = new OneShot();
    OneShot spinner = new OneShot();

    OneShot directionSwitch = new OneShot();





    //initialize
    @Override
    public void init() {
        telemetry.addData("INITIALIZING","");
        telemetry.update();
        robot.init(hardwareMap);
        telemetry.addData("Done Initializing","");
        telemetry.update();
    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    @Override
    public void loop() {

        double leftPower  = -gamepad1.left_stick_y ;
        double rightPower = -gamepad1.right_stick_y ;
        double rightLiftPower = -gamepad2.left_stick_y;

        //boolean Flicker = gamepad2.right_bumper;


        robot.setChassisTankDrivePower(leftPower, rightPower);
        robot.boxMove(rightLiftPower);

        if (gamepad2.right_trigger > 0){
            robot.startRoller();
        }
        else{
            robot.stopRoller();
        }

        //if (IntakeArm.checkState(gamepad2.left_bumper)){
            //robot.flick();
        //}

        if (gamepad2.left_trigger > 0){
            robot.reverseRoller();
        }

        if (dumpCargoCheck.checkState(gamepad2.b)){
            robot.dumpCargo();
        }

        if (secureCargoCheck.checkState(gamepad2.a)){
            robot.secureCargo();
        }

        if (directionSwitch.checkState(gamepad1.a)){
            robot.directionSwitch();
        }

        if (resetBox.checkState(gamepad2.x)){
            robot.resetBox();
        }

        /*
        if (shortlift.checkState(gamepad2.dpad_down)){
            robot.raiseCargoLift(4);
        }

         */

        //telemetry.addData("Direction", robot.debug());
        //telemetry.addData("Duck Position", robot.getDuckPosition());
        //telemetry.update();

        if (gamepad2.left_bumper) {
            robot.startCarouselSpinner();
        }
        else if (gamepad2.right_bumper){
            robot.startReverseSpinnerSpinner();
        }else {
            robot.stopCarouselSpinnerSpinner();
        }

        robot.update();

        telemetry.update();

    }
}
