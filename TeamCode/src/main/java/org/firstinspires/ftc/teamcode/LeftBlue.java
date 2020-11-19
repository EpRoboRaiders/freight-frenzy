package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="LeftBlue", group="AutonomousBase")
// @Disabled

public class LeftBlue extends AutonomousBase {

    public void runOpMode() {

        initialize();

        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Position", pipeline.position);
        telemetry.update();

        sleep(2000);

        if (pipeline.position == RingStackMeasurerPipeline.RingPosition.NONE) {
            // Placement A2
            positionA2();
        } else if (pipeline.position == RingStackMeasurerPipeline.RingPosition.ONE) {
            // Placement B
            positionB();

        } else if (pipeline.position == RingStackMeasurerPipeline.RingPosition.FOUR) {
            // Placement C (Default)
            positionC();
        }
    }

    public void positionA2() {
        // Wobble Goal Placement A2
        encoderDrive(DRIVE_SPEED, 60, 60, 60, 60, 5);

        //pause before and after first turn
        rotate(TURN_SPEED, 40);

        encoderDrive(DRIVE_SPEED, 12, 12, 12, 12, 5);

        encoderDrive(DRIVE_SPEED, -10, -10, -10, -10, 5);

        rotate(TURN_SPEED,-43);

        // moves faster to the wall
        encoderDrive(DRIVE_SPEED,-45,-45,-45,-45,5);

        // slows down before it hits the wall
        //pauses before going backwards
        encoderDrive(.1, -20, -20, -20, -20, 3);

        //strafe left to the corner
        encoderDrive(.1,-24,24,24,-24,5);

        //back up so robot straightens out
        encoderDrive(.1,-2.5,-2.5,-2.5,-2.5, 5);

        //strafe right
        encoderDrive(.1,44.5,-44.5,-44.5,44.5,7);

        //move forward to grab wobble
        encoderDrive(.1, 10,10,10,10,5);

        //strafe left
        encoderDrive(DRIVE_SPEED,-38,38,38,-38,7);

        //move forward to the box
        encoderDrive(DRIVE_SPEED, 46, 46, 46, 46, 5);

        // pulls out of the box
        encoderDrive(DRIVE_SPEED,-6,-6,-6,-6,5);

        // strafes over right
        encoderDrive(DRIVE_SPEED, 20, -20, -20, 20, 5);

        // park on white line
        encoderDrive(DRIVE_SPEED, 10, 10, 10,10, 5);
    }

    public void positionB() {
        // Wobble Goal Placement B
        encoderDrive(DRIVE_SPEED, 80, 80, 80, 80, 5);

        rotate(DRIVE_SPEED, -45);

        encoderDrive(DRIVE_SPEED, 12,12, 12,12, 5);

        encoderDrive(DRIVE_SPEED, -10, -10, -10, -10, 5);
    }

    public void positionC() {
        // Wobble Goal Placement C
        encoderDrive(DRIVE_SPEED, 115, 115, 115, 115, 10);

        encoderDrive(DRIVE_SPEED, -6,6,6,-6,5);

        encoderDrive(DRIVE_SPEED, -30, -30, -30, -30, 5);
    }
}

