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

        //moves forward to put wobble goal in box
        encoderDrive(DRIVE_SPEED, 80, 80, 80, 80, 5);

        //strafes to put wobble goal in box
        //changed the distance so the robot does not move as far into the box
        encoderDrive(DRIVE_SPEED, 10, -10, -10, 10, 5);

        //moves forward into box
        //does not move as far into the box
        encoderDrive(DRIVE_SPEED, 10, 10, 10, 10, 5);

        //pulls out of box
        encoderDrive(DRIVE_SPEED, -8, -8, -8, -8, 5);

        //strafes toward wall
        encoderDrive(DRIVE_SPEED, -20, 20, 20, -20, 5);

        //moves back to the starting wall
        encoderDrive(.1, -82, -82, -82, -82, 5);

        //strafes over into the corner
        //was 28 is now 2, so it moves 26 less than before
        encoderDrive(.1, -2, 2, 2, -2, 5);

        //moves back into corner to straighten out
        encoderDrive(.1, -4, -4, -4, -4, 5);

        //strafes over to grab wobble gaol
        encoderDrive(.1, 43, -43, -43, 43, 5);

        //moves forward to grab wobble goal
        encoderDrive(DRIVE_SPEED, 7, 7, 7, 7, 5);

        //strafes over right to get out of the way of the ring
        encoderDrive(DRIVE_SPEED, 15, -15,-15,15,5);

        //moves to the box
        //not far enough so it is now 8 inches farther into the box
        encoderDrive(DRIVE_SPEED, 78, 78, 78, 78, 5);

        //strafes into the box
        encoderDrive(DRIVE_SPEED, -10, 10, 10, -10, 5);

        //pulls out of box and parks on the white line
        encoderDrive(DRIVE_SPEED, -15,-15, -15,-15, 5);
    }

    public void positionC() {
        // Wobble Goal Placement C

        encoderDrive(DRIVE_SPEED, 104, 104, 104, 104, 5);

        encoderDrive(DRIVE_SPEED, -8, 8, 8, -8, 5);

        encoderDrive(DRIVE_SPEED, 8, 8, 8, 8, 5);

        encoderDrive(DRIVE_SPEED, -104, -104, -104, -104, 5);

        // Ram into the wall.
        encoderDrive(.1, -14, 14, 14, -14, 5);

        encoderDrive(.1, -10, -10, -10, -10, 5);

        encoderDrive(.1, 43, -43, -43, 43, 5);
        // End ram

        encoderDrive(DRIVE_SPEED, 5, 5, 5, 5, 5);

        encoderDrive(DRIVE_SPEED, -35, 35, 35, -35, 5 );

        encoderDrive(DRIVE_SPEED, 96, 96, 96, 96, 5);

        encoderDrive(DRIVE_SPEED, -13, 13, 13, -13, 5);

        encoderDrive(DRIVE_SPEED, -30, -30, -30, -30, 5);

    }
}