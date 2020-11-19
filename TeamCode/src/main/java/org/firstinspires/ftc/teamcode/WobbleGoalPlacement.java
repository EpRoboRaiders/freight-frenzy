package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "WobbleGoal", group = "Autonomous")
//@Disabled
public class WobbleGoalPlacement extends AutonomousBase {

    static final double FORWARD_SPEED = 0.3;

    @Override
    public void runOpMode(){

        initialize();

        encoderDrive(DRIVE_SPEED, 80, 80, 80, 80, 5);

        rotate(DRIVE_SPEED, -45);

        encoderDrive(DRIVE_SPEED, 12,12, 12,12, 5);

        encoderDrive(DRIVE_SPEED, -10, -10, -10, -10, 5);
    }
}


