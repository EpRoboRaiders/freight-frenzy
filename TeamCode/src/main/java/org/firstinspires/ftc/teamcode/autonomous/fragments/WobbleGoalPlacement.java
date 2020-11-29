package org.firstinspires.ftc.teamcode.autonomous.fragments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousBase;

@Autonomous(name = "WobbleGoal", group = "Autonomous")
//@Disabled
public class WobbleGoalPlacement extends AutonomousBase {

    static final double FORWARD_SPEED = 0.3;

    @Override
    public void runOpMode(){

        initialize();

        encoderDrive(DRIVE_SPEED, 80, 80, 80, 80, 5);

        encoderDrive(DRIVE_SPEED, 16, -16, -16, 16, 5);

        encoderDrive(DRIVE_SPEED, 8, 8, 8, 8, 5);

        encoderDrive(DRIVE_SPEED, -8, -8, -8, -8, 5);

        encoderDrive(DRIVE_SPEED, -18, 18, 18, -18, 5);

        encoderDrive(.1, -80, -80, -80, -80, 5);

        encoderDrive(.1, -28, 28, 28, -28, 5);

        encoderDrive(.1, -4, 4, -4, -4, 5);

        encoderDrive(.1, 42.5, -42.5, -42.5, 42.5, 5);

        encoderDrive(DRIVE_SPEED, 85, 85, 85, 85, 5);

        encoderDrive(.1, -3, 3, 3, -3, 5);

        encoderDrive(DRIVE_SPEED, -15,-15, -15,-15, 5);

    }
}


