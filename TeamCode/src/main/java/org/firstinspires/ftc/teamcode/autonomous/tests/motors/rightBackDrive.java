package org.firstinspires.ftc.teamcode.autonomous.tests.motors;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousBase;

@Autonomous(name = "rightBackDrive", group = "Autonomous")
//@Disabled
public class rightBackDrive extends AutonomousBase {

    @Override
    public void runOpMode(){

        initialize();

        robot.rightBackDrive.setPower(DRIVE_SPEED);

        sleep(30000);
    }
}