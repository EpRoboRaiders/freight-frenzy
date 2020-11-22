package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

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
