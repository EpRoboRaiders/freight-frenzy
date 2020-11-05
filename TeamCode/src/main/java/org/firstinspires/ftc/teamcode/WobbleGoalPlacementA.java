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

        encoderDrive(.1, 60, 60, 60, 60, 5);

        rotate(.1, 45);

        encoderDrive(.1, 8, 8, 8, 8, 5);

        encoderDrive(.1, -19, -19, -19, -19, 5);

        rotate(.1, -45);

        encoderDrive(.1, 20, 20, 20, 20, 5);
    }
}
