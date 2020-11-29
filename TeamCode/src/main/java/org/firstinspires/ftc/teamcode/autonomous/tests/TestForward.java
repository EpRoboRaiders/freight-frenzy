package org.firstinspires.ftc.teamcode.autonomous.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousBase;

@Autonomous(name = "ForwardTest", group = "Autonomous")
//@Disabled
public class TestForward extends AutonomousBase {

    @Override
    public void runOpMode(){

        initialize();

        robot.leftFrontDrive.setPower(DRIVE_SPEED);
        robot.leftBackDrive.setPower(DRIVE_SPEED);
        robot.rightFrontDrive.setPower(DRIVE_SPEED);
        robot.rightBackDrive.setPower(DRIVE_SPEED);

        sleep(30000);
    }
}
