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

        encoderDrive(DRIVE_SPEED, 115, 115, 115, 115, 10);

        encoderDrive(DRIVE_SPEED, -6,6,6,-6,5);

        encoderDrive(DRIVE_SPEED, -30, -30, -30, -30, 5);

    }
}
