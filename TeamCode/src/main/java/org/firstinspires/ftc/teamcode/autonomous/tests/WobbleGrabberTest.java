package org.firstinspires.ftc.teamcode.autonomous.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousBase;

@Autonomous(name = "GrabberTest", group = "Autonomous")
//@Disabled
public class WobbleGrabberTest extends AutonomousBase {

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();

        robot.wobbleGrabber.lowerAndOpen();

        sleep(3000);

        robot.wobbleGrabber.closeAndRaise();

    }

}
