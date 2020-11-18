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

        //pause before and after first turn
        rotate(TURN_SPEED, 40);

        encoderDrive(DRIVE_SPEED, 12, 12, 12, 12, 5);

        encoderDrive(DRIVE_SPEED, -10, -10, -10, -10, 5);

        rotate(TURN_SPEED,-43);

        // moves faster to the wall
        encoderDrive(DRIVE_SPEED,-45,-45,-45,-45,5);

        // slows down before it hits the wall
        //pauses before going backwards
        encoderDrive(.1, -20, -20, -20, -20, 3);

        //strafe left to the corner
        encoderDrive(.1,-24,24,24,-24,5);

        //back up so robot straightens out
        encoderDrive(.1,-2.5,-2.5,-2.5,-2.5, 5);

        //strafe right
        encoderDrive(.1,44.5,-44.5,-44.5,44.5,7);

        //move forward to grab wobble
        encoderDrive(.1, 10,10,10,10,5);

        //strafe left
        encoderDrive(DRIVE_SPEED,-38,38,38,-38,7);

        // slows down before it hits the wall
        //encoderDrive(.1, -18, 18, 18, -18, 5);

        //move forward to the box
        encoderDrive(DRIVE_SPEED, 46, 46, 46, 46, 5);

        // pulls out of the box
        encoderDrive(DRIVE_SPEED,-6,-6,-6,-6,5);

        // strafes over right
        encoderDrive(DRIVE_SPEED, 20, -20, -20, 20, 5);

        // park on white line
        encoderDrive(DRIVE_SPEED, 10, 10, 10,10, 5);

    }
}
