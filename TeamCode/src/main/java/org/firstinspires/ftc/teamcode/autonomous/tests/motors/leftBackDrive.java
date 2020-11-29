package org.firstinspires.ftc.teamcode.autonomous.tests.motors;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousBase;

@Autonomous(name = "leftBackDrive", group = "Autonomous")
//@Disabled
public class leftBackDrive extends AutonomousBase {

    @Override
    public void runOpMode(){

        initialize();

        robot.leftBackDrive.setPower(DRIVE_SPEED);

        sleep(30000);
    }
}
