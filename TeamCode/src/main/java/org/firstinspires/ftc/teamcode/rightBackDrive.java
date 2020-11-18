package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "rightBackDrive", group = "Autonomous")
@Disabled
public class rightBackDrive extends AutonomousBase {

    @Override
    public void runOpMode(){

        initialize();

        robot.rightBackDrive.setPower(DRIVE_SPEED);

        sleep(30000);
    }
}