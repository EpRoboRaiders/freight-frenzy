package org.firstinspires.ftc.teamcode.Mecanum;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name= "Mecanum Drive", group = "TeleOp")
public class MecanumDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor leftFrontDrive = hardwareMap.dcMotor.get("left_front_drive");
        DcMotor leftBackDrive = hardwareMap.dcMotor.get("left_back_drive");
        DcMotor rightFrontDrive = hardwareMap.dcMotor.get("right_front_drive");
        DcMotor rightBackDrive = hardwareMap.dcMotor.get("right_back_drive");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);

        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            double ry = gamepad1.right_stick_y;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            /*double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;*/

            double frontLeftPower = 0;
            double frontRightPower = 0;
            double backLeftPower = 0;
            double backRightPower = 0;

            if ( (x < 0.1) && (x > -0.1)) {
                frontLeftPower = Range.clip(y,-1.0, 1.0);
                frontRightPower = Range.clip(y,-1.0, 1.0);
                backRightPower = Range.clip(y,-1.0, 1.0);
                backLeftPower = Range.clip(y,-1.0, 1.0);
            }

            if ((y < 0.1) && (y > -0.1)) {
                frontLeftPower = x;
                frontRightPower = -x;
                backLeftPower = -x;
                backRightPower = x;
            }

            //top left side strafe      done
            if ((x < -0.1) && (y > 0.1)) {
                frontLeftPower = 0;
                frontRightPower = -x;
                backLeftPower = -x;
                backRightPower = 0;
            }

            //bottom left side strafe     done
            if ((x < -0.1) && (y < -0.1)) {
                frontLeftPower = x;
                frontRightPower = 0;
                backLeftPower = 0;
                backRightPower = x;
            }

            //top right side strafe      done
            if ((x > 0.1) && (y > 0.1)) {
                frontLeftPower = x;
                frontRightPower = 0;
                backLeftPower = 0;
                backRightPower = x;
            }

            //bottom right side strafe
            if ((x > 0.1) && (y < -0.1)) {
                frontLeftPower = 0;
                frontRightPower =   -x;
                backLeftPower = -x;
                backRightPower = 0;
            }

            if ((rx > 0.1) || (rx < -0.1)){
                frontLeftPower = rx;
                frontRightPower = -rx;
                backLeftPower = rx;
                backRightPower = -rx;
            }


            leftFrontDrive.setPower(frontLeftPower);
            leftBackDrive.setPower(backLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            rightBackDrive.setPower(backRightPower);

            telemetry.addData("front left", frontLeftPower);
            telemetry.addData("front right", frontRightPower);
            telemetry.addData("back left", backLeftPower);
            telemetry.addData("back right", backRightPower);

            telemetry.addData("Y Information", y);
            telemetry.addData("X Information", x);
            telemetry.update();
        }
    }
}
