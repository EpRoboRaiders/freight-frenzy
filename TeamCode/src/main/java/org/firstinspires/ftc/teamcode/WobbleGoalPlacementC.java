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

        encoderDrive(.1, 109, 109, 109, 109, 10);

        rotate(.1, 45);

        encoderDrive(.1, 8, 8, 8, 8, 5);

        encoderDrive(.1, -18, -18, -18, -18, 5);

        rotate(.1, -45);

        encoderDrive(.1, -30, -30, -30, -30, 5);

    }
}
