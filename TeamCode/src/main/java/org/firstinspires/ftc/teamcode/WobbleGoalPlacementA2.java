package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "WobbleGoalA2", group = "Autonomous")
//@Disabled
public class WobbleGoalPlacementA2 extends AutonomousBase {

    @Override
    public void runOpMode(){

        initialize();

        encoderDrive(DRIVE_SPEED, 60, 60, 60, 60, 5);

        rotate(DRIVE_SPEED, 45);

        encoderDrive(DRIVE_SPEED, 10, 10, 10, 10, 5);

        encoderDrive(DRIVE_SPEED, -10, -10, -10, -10, 5);

        rotate(DRIVE_SPEED,-43);

        encoderDrive(.1,-65,-65,-65,-65,5);

        //strafe left to the corner
        encoderDrive(.1,-19,19,19,-19,5);

        //strafe right
        encoderDrive(.1,44,-44,-44,44,7);

        //move forward to grab wobble
        encoderDrive(.1, 10,10,10,10,5);

        //strafe left
        encoderDrive(.1,-45.2,45.2,45.2,-45.2,7);

        //repeat first 5 steps
        encoderDrive(.1, 48, 48, 48, 48, 5);
/*
        rotate(.1, 45);

        encoderDrive(.1, 8, 8, 8, 8, 5);

        encoderDrive(.1, -19, -19, -19, -19, 5);

        rotate(.1,-45);

        //park on white line
        encoderDrive(.1,10,10,10,10,5);
         */
    }
}
