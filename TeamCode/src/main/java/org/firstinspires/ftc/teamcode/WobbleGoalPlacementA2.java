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

        encoderDrive(.1, 60, 60, 60, 60, 5);

        rotate(.1, 45);

        encoderDrive(.1, 10, 10, 10, 10, 5);

        encoderDrive(.1, -21, -21, -21, -21, 5);

        rotate(.1,-45);

        encoderDrive(.1,-65,-65,-65,-65,5);

        //strafe left to the corner
        encoderDrive(.1,-30,30,30,-30,5);

        //strafe right
        encoderDrive(.1,45.2,-45.2,-45.2,45.2,7);

        //move forward to grab wobble
        encoderDrive(.1, 10,10,10,10,5);

        //strafe left
        encoderDrive(.1,-45.2,45.2,45.2,-45.2,7);

        //repeat first 5 steps
        encoderDrive(.1, 60, 60, 60, 60, 5);

        rotate(.1, 45);

        encoderDrive(.1, 8, 8, 8, 8, 5);

        encoderDrive(.1, -19, -19, -19, -19, 5);

        rotate(.1,-45);

        //park on white line
        encoderDrive(.1,10,10,10,10,5);
    }
}
