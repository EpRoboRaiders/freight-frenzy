package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "StrafeTest", group = "Autonomous")
//@Disabled
public class StrafeRight extends AutonomousBase {

    @Override
    public void runOpMode(){

        initialize();

        robot.leftFrontDrive.setPower(DRIVE_SPEED);
        robot.leftBackDrive.setPower(-DRIVE_SPEED*.9);
        robot.rightFrontDrive.setPower(-DRIVE_SPEED);
        robot.rightBackDrive.setPower(DRIVE_SPEED*.9);

        sleep(30000);
    }
}
