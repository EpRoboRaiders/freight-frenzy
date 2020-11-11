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

        encoderDrive(DRIVE_SPEED, 109, 109, 109, 109, 10);

        rotate(DRIVE_SPEED, 45);

        encoderDrive(DRIVE_SPEED, 8, 8, 8, 8, 5);

        encoderDrive(DRIVE_SPEED, -18, -18, -18, -18, 5);

        rotate(DRIVE_SPEED, -45);

        encoderDrive(DRIVE_SPEED, -30, -30, -30, -30, 5);

    }
}
