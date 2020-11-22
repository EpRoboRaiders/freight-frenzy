package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

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