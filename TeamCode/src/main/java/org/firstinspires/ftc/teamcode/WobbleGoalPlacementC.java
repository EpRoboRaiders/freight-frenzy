package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "WobbleGoalC", group = "Autonomous")
//@Disabled
public class WobbleGoalPlacementC extends AutonomousBase {

    @Override
    public void runOpMode(){

        initialize();

        encoderDrive(DRIVE_SPEED, 104, 104, 104, 104, 5);

        encoderDrive(DRIVE_SPEED, -8, 8, 8, -8, 5);

        encoderDrive(DRIVE_SPEED, 8, 8, 8, 8, 5);

        encoderDrive(DRIVE_SPEED, -104, -104, -104, -104, 5);


        // Ram into the wall.
        encoderDrive(.1, -12, 12, 12, -12, 5);

        encoderDrive(.1, -8, -8, -8, -8, 5);

        encoderDrive(.1, 42.5, -42.5, -42.5, 42.5, 5);
        // End ram





        encoderDrive(DRIVE_SPEED, 100, 100, 100, 100, 5);

        encoderDrive(DRIVE_SPEED, -35, 35, 35, -35, 5);

        encoderDrive(DRIVE_SPEED, 8, 8, 8, 8, 5);

        encoderDrive(DRIVE_SPEED, -30, -30, -30, -30, 5);






    }
}
