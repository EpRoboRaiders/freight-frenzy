package org.firstinspires.ftc.teamcode.autonomous.tests.motors;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousBase;

@Autonomous(name = "rightFrontDrive", group = "Autonomous")
//@Disabled
public class rightFrontDrive extends AutonomousBase {

    @Override
    public void runOpMode(){

        initialize();

        robot.rightFrontDrive.setPower(DRIVE_SPEED);

        sleep(30000);
    }
}