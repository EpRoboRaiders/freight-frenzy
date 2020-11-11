package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "WobbleGoalA", group = "Autonomous")
//@Disabled
public class WobbleGoalPlacementA extends AutonomousBase {

    @Override
    public void runOpMode(){

        initialize();

        encoderDrive(DRIVE_SPEED, 60, 60, 60, 60, 5);

        rotate(DRIVE_SPEED, 45);

        encoderDrive(DRIVE_SPEED, 8, 8, 8, 8, 5);

        encoderDrive(DRIVE_SPEED, -19, -19, -19, -19, 5);

        rotate(DRIVE_SPEED, -45);

        encoderDrive(DRIVE_SPEED, 10, 10, 10, 10, 5);
    }
}
