package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="LeftBlue", group="AutonomousBase")
// @Disabled

public class LeftBlue extends AutonomousBase {
    public void runOpMode() {

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        encoderDrive(.2, 18, 18, 18, 18, 3);
        encoderDrive(.2, -12, -12, -12, -12, 3);


    }
}
