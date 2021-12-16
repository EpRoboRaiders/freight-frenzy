package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constructers.BaseRobot;

@Autonomous(name = "Red Encoder Autonomous", group = "Autonomous")
public class RedEncoderAutonomous extends LinearOpMode
{
    private BaseRobot robot = new BaseRobot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);


        waitForStart();

        robot.secureCargo();

        robot.raiseCargoLift(4);

        robot.chassisDrive(BaseRobot.DRIVE_SPEED,-10,-10,5);

        robot.chassisDrive(BaseRobot.DRIVE_SPEED,-8,8,5);

        robot.chassisDrive(BaseRobot.DRIVE_SPEED,-17,-17,5);

        robot.raiseCargoLift(24);

        robot.dumpCargo();

        sleep(1000);

        robot.resetBox();

        robot.chassisDrive(BaseRobot.DRIVE_SPEED,4,4,5);

        robot.chassisDrive(BaseRobot.DRIVE_SPEED,-8,8,5);

        robot.chassisDrive(BaseRobot.DRIVE_SPEED,57,57,5);

        robot.lowerCargoLift(24);

    }
}
