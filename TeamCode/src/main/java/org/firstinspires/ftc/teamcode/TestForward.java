package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "ForwardTest", group = "Autonomous")
@Disabled
public class TestForward extends AutonomousBase {

    @Override
    public void runOpMode(){

        initialize();

        robot.leftFrontDrive.setPower(DRIVE_SPEED);
        robot.leftBackDrive.setPower(DRIVE_SPEED);
        robot.rightFrontDrive.setPower(DRIVE_SPEED);
        robot.rightBackDrive.setPower(DRIVE_SPEED);

        sleep(1000);

        robot.leftFrontDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
    }
}
