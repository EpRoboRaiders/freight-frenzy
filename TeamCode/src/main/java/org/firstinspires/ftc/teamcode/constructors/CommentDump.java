package org.firstinspires.ftc.teamcode.constructors;

public class CommentDump {


    // The following is COMPLETELY COSMETIC; don't touch unless you hate fun.

    //boolean moyesFound;
    //int moyesSoundID;
    //boolean leftStickPressed = false;

     /*
        // Also cosmetic
        int moyesSoundID = hardwareMap.appContext.getResources().getIdentifier("moyes", "raw", hardwareMap.appContext.getPackageName());

        if (moyesSoundID != 0)
            moyesFound   = SoundPlayer.getInstance().preload(hardwareMap.appContext, moyesSoundID);

         */
         /*
        if (gamepad1.y != y1Pressed) {

            if(!y1Pressed) {
                mode = mode.getNext();
            }
            y1Pressed = !y2Pressed;
        }

         */

    // Controller Comments

    // Map the clampRotator to the x axis of the second gamepad's right joystick.
    // robot.clampRotator.setPosition(gamepad2.right_stick_x);

    // robot.ringClamp.setPosition(gamepad2.right_stick_x);

    //robot.wobbleGrabber.setPosition(gamepad2.right_stick_x);
    //gamepad2.left_stick_x

    // If the X button is pressed, activate the shooter arm and the shooter mechanism itself.


        /*
        if(gamepad2.x) {
            robot.leftShooter.setPower(.71);
            robot.rightShooter.setPower(.71);
            robot.shooterArm.setPosition(1);
        }
        // Less powerful shot for the Power Targets.
        else if(gamepad2.left_trigger > .2) {
            robot.leftShooter.setPower(.61);
            robot.rightShooter.setPower(.61);
            robot.shooterArm.setPosition(1);
        }
        else {
            robot.leftShooter.setPower(0);
            robot.rightShooter.setPower(0);
            robot.shooterArm.setPosition(.8);
        }

         */





    //robot.wobbleGrabber.setPosition(gamepad2.right_stick_x);
        /*
        if (gamepad2.left_trigger > 0) {
            robot.grabberArm.setPower(0.06);
        }
        else {
            robot.grabberArm.setPower(-gamepad2.left_stick_y);
        }
         */

        /*
        // Also cosmetic
        if (gamepad2.left_stick_button && !leftStickPressed) {
            leftStickPressed = !leftStickPressed;
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, moyesSoundID);

        }
        if (!gamepad2.left_stick_button && leftStickPressed) {
            leftStickPressed = !leftStickPressed;
        }

         */

    // telemetry.addData("Motor Position:", robot.leftFrontDrive.getCurrentPosition());

// telemetry.addData("Left Front Motor: ", robot.leftFrontDrive.getPower());
// telemetry.addData("Right Front Motor: ", robot.rightFrontDrive.getPower());
// telemetry.addData("Left Back Motor: ", robot.leftBackDrive.getPower());
// telemetry.addData("Fight Back Motor: ", robot.rightBackDrive.getPower());

// telemetry.addData("Wobble Grabber Servo: ", robot.wobbleGrabber.getPosition());
// telemetry.addData("Grabber Arm Servo: ", robot.grabberArm.getPosition());
// telemetry.addData("Ring Clamp: ", robot.ringClamp.getPosition());
// telemetry.addData("Clamp Rotator: ", robot.clampRotator.getPosition());
// telemetry.addData("intakeArm :", robot.intakeArm.getCurrentPosition());

// telemetry.addData("looptime :", looptime.milliseconds());
// telemetry.addData("gold resource",   moyesFound ?   "Found" : "NOT found\n Add moyes.wav to /src/main/res/raw" );
}
