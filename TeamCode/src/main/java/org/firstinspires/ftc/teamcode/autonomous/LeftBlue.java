package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.constructors.CPipeline;

@Autonomous(name="LeftBlue", group="AutonomousBase")
// @Disabled

public class LeftBlue extends AutonomousBase {


    public void runOpMode() {

        initialize();

        telemetry.addData("Analysis", robot.webcam.pipeline.getAnalysis());
        telemetry.addData("Position", robot.webcam.pipeline.getRingAmount());
        telemetry.update();

        sleep(2000);

        if (robot.webcam.pipeline.getRingAmount() == CPipeline.RingPosition.NONE) {
            // Placement A2
            positionA2();
            //shooterPosition();
        } else if (robot.webcam.pipeline.getRingAmount() == CPipeline.RingPosition.ONE) {
            // Placement B
            positionB();
            //shooterPosition();
        } else if (robot.webcam.pipeline.getRingAmount() == CPipeline.RingPosition.FOUR) {
            // Placement C (Default)
            positionC();
            //shooterPosition();
        }
    }

    public void positionA2() {
        // Wobble Goal Placement A2
        encoderDrive(DRIVE_SPEED, 60, 60, 60, 60, 5);

        //strafe into the box
        //was 10, now it is 15
        encoderDrive(DRIVE_SPEED,-15, 15, 15, -15, 5);

        //pulls out of box and back to the wall
        encoderDrive(DRIVE_SPEED, -45, -45, -45, -45, 5);

        // slows down before it hits the wall
        encoderDrive(.1, -20, -20, -20, -20, 3);

        /*
        //strafe left to the corner
        //was 24, now it is 12
        encoderDrive(.1,-12,12,12,-12,5);
        */

        //back up so robot straightens out
        //was 2.5, now it is 1
        encoderDrive(.1,-1,-1,-1, -1, 5);

        //strafe right
        //was 44, now it is 43
        encoderDrive(.1,43,-43,-43,43,7);

        //move forward to grab wobble
        encoderDrive(.1, 10,10,10,10,5);

        //strafe left
        //was 38, mow it is 45
        encoderDrive(DRIVE_SPEED,-42,42,42,-42,7);

        //move forward to the box
        //was 46, now it is 40
        encoderDrive(DRIVE_SPEED, 40, 40, 40, 40, 5);

        //strafes farther into the box
        encoderDrive(DRIVE_SPEED, -10, 10, 10, -10, 5);

        // pulls out of the box
        encoderDrive(DRIVE_SPEED,-6,-6,-6,-6,5);

        // strafes over right
        encoderDrive(DRIVE_SPEED, 20, -20, -20, 20, 5);

        // park on white line
        //was 10, now it is 15
        encoderDrive(DRIVE_SPEED, 15, 15, 15,15, 5);
    }

    public void positionB() {
        // Wobble Goal Placement B

        //moves forward to put wobble goal in box
        encoderDrive(DRIVE_SPEED, 80, 80, 80, 80, 5);

        //strafes to put wobble goal in box
        //changed the distance by 2 inches so the robot will not move farther into the box
        encoderDrive(DRIVE_SPEED, 13, -13, -13, 13, 5);

        //moves forward into box
        //does not move as far into the box
        encoderDrive(DRIVE_SPEED, 10, 10, 10, 10, 5);

        //pulls out of box
        encoderDrive(DRIVE_SPEED, -8, -8, -8, -8, 5);

        //strafes toward wall
        encoderDrive(DRIVE_SPEED, -22.5, 22.5, 22.5, -22.5
                , 5);

        //moves back to the starting wall
        encoderDrive(.1, -82, -82, -82, -82, 5);

        //strafes over into the corner
        //was 28 is now 10, so it moves 18 less than before
        encoderDrive(.1, -10, 10, 10, -10, 2.5);

        //moves back into corner to straighten out
        encoderDrive(.1, -4, -4, -4, -4, 2.5);

        //strafes over to grab wobble gaol
        encoderDrive(.1, 42, -42, -42, 42, 5);

        //moves forward to grab wobble goal
        encoderDrive(DRIVE_SPEED, 7, 7, 7, 7, 5);

        //strafes over right to get out of the way of the ring
        encoderDrive(DRIVE_SPEED, 15, -15,-15,15,5);

        //moves to the box
        //not far enough so it is now 8 inches farther into the box
        encoderDrive(DRIVE_SPEED, 78, 78, 78, 78, 5);

        /*
        //strafes into the box
        //was 10 inches now it is 7
        encoderDrive(DRIVE_SPEED, -7, 7, 7, -7, 5);
         */

        //pulls out of box and parks on the white line
        //was 15 now it is ten
        encoderDrive(DRIVE_SPEED, -10,-10, -10,-10, 5);
    }

    public void positionC() {
        // Wobble Goal Placement C

        //moves forward to put wobble goal in box
        encoderDrive(DRIVE_SPEED, 104, 104, 104, 104, 5);

        //strafes into box
        //changed the distance so now it moves 4 more inches over
        encoderDrive(DRIVE_SPEED, -12, 12, 12, -12, 5);

        //moves forward into box
        encoderDrive(DRIVE_SPEED, 8, 8, 8, 8, 5);

        //pulls out of box and back to the starting wall.
        encoderDrive(DRIVE_SPEED, -104, -104, -104, -104, 5);

        // Ram into the wall.
        encoderDrive(.1, -14, 14, 14, -14, 5);

        encoderDrive(.1, -10, -10, -10, -10, 5);

        encoderDrive(.1, 43, -43, -43, 43, 5);
        // End ram

        encoderDrive(DRIVE_SPEED, 5, 5, 5, 5, 5);

        encoderDrive(DRIVE_SPEED, -35, 35, 35, -35, 5 );

        //turn left so that the robot straightens out
        //was 7 degree turn, now it is a 6 degree turn
        rotate(TURN_SPEED, 6);

        encoderDrive(DRIVE_SPEED, 96, 96, 96, 96, 5);

        encoderDrive(DRIVE_SPEED, -30, -30, -30, -30, 5);

    }
}