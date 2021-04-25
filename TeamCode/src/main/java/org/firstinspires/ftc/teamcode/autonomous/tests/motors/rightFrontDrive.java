package org.firstinspires.ftc.teamcode.autonomous.tests.motors;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousBase;
import org.firstinspires.ftc.teamcode.constructors.AutonomousTemplate;
import org.firstinspires.ftc.teamcode.constructors.TeleOpTemplate;

@Autonomous(name = "rightFrontDrive", group = "Autonomous")
//@Disabled
public class rightFrontDrive extends AutonomousBase {
    private TeleOpTemplate robot = new TeleOpTemplate();
    @Override
    public void runOpMode(){

        robot.init(hardwareMap);

        telemetry.addData("Status", "Camera initializing; please wait (~5 seconds)");
        telemetry.update();
        sleep(5000);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        waitForStart();

        robot.drivetrain.rightFrontDrive.setPower(DRIVE_SPEED);

        while (true) {
            telemetry.addData("encoder count", robot.drivetrain.rightFrontDrive.getCurrentPosition());
            telemetry.update();
            sleep(1000);
        }

        //sleep(30000);
    }
}