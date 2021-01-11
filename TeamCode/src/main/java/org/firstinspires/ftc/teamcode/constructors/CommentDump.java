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



    // NewLeftBlue (ring shooter)
    /*
        ArrayList<Trajectory> trajectories = new ArrayList<Trajectory>();

        trajectories.add(robot.drive.trajectoryBuilder(new Pose2d())
                //first trajectory moves the first wobble goal into box C
                //every time the robot stops, a new trajectory must be made
                .strafeRight(6)
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size() - 1).end())
                //second trajectory moves the robot to line up with the second wobble goal
                .strafeRight(6)
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size() - 1).end())
                //moves forward to grab the wobble goal
                //.forward(14)
                .strafeRight(6)
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size() - 1).end())
                //moves the wobble goal into box C
                .forward(8)
                .build());

        robot.ringIntake.extendIntake();

        robot.drive.followTrajectory(trajectories.get(0));

        robot.ringShooter.powerShot();

        robot.drive.followTrajectory(trajectories.get(1));

        robot.ringShooter.powerShot();

        robot.drive.followTrajectory(trajectories.get(2));

        robot.ringShooter.powerShot();

        robot.drive.followTrajectory(trajectories.get(3));

        //robot.ringShooter.autonomousTowerShot();

        robot.ringIntake.retractIntake();

         */



        /*
        robot.ringShooter.towerShot();
        robot.hopperLifter.setPosition(0.09);

        sleep(500);

        robot.intakeArm.setPower(0);
        robot.leftShooter.setPower(.71);
        robot.rightShooter.setPower(.71);
        robot.shooterArm.setPosition(1);

        sleep(500);

        robot.leftShooter.setPower(0);
        robot.rightShooter.setPower(0);
        robot.shooterArm.setPosition(.75);

        sleep(100);
        robot.hopperLifter.setPosition(0.04);

        sleep(1000);

        robot.leftShooter.setPower(.71);
        robot.rightShooter.setPower(.71);
        robot.shooterArm.setPosition(1);

        sleep(500);

        robot.leftShooter.setPower(0);
        robot.rightShooter.setPower(0);
        robot.shooterArm.setPosition(.8);

        sleep(100);
        robot.hopperLifter.setPosition(0);

        sleep(1000);

        robot.leftShooter.setPower(.71);
        robot.rightShooter.setPower(.71);
        robot.shooterArm.setPosition(1);

        sleep(500);

        robot.leftShooter.setPower(0);
        robot.rightShooter.setPower(0);
        robot.shooterArm.setPosition(.8);
        robot.intakeArm.setPower(.6);

        sleep(1500);

        robot.intakeArm.setPower(0);
        robot.hopperLifter.setPosition(0.15);

         */
}
