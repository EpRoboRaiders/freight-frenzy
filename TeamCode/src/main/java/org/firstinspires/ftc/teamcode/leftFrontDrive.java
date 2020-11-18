package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "leftFrontDrive", group = "Autonomous")
@Disabled
public class leftFrontDrive extends AutonomousBase {

    @Override
    public void runOpMode(){

        initialize();

        robot.leftFrontDrive.setPower(DRIVE_SPEED);

        sleep(30000);
    }
}
