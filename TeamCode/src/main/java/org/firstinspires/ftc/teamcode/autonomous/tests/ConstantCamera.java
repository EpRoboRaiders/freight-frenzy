package org.firstinspires.ftc.teamcode.autonomous.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousBase;
import org.firstinspires.ftc.teamcode.constructors.AutonomousTemplate;
import org.firstinspires.ftc.teamcode.constructors.CPipeline;
import java.util.ArrayList;


@Autonomous(name = "ConstantCamera", group = "Tests")
public class ConstantCamera extends AutonomousBase {
    private AutonomousTemplate robot = new AutonomousTemplate();

    //private final double POWER_SHOT_TURN_TO_POSITION_A =   0 ; //-10.57;
    //private final double POWER_SHOT_TURN_TO_POSITION_B =  -5 ; // -16.35 + 10.57;
    //private final double POWER_SHOT_TURN_TO_POSITION_C =  -10; //-21.80 + 5;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Camera initializing; please wait (~5 seconds)");
        telemetry.update();
        sleep(5000);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        waitForStart();
        robot.ringShooter.teleOpInit();

        while (opModeIsActive()) {
            telemetry.addData("Analysis", robot.webcam.pipeline.getAnalysis());
            telemetry.addData("Position", robot.webcam.pipeline.getRingAmount());
            telemetry.update();

            sleep(3000);
        }
    }
}