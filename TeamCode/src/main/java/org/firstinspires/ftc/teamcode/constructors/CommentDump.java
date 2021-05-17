package org.firstinspires.ftc.teamcode.constructors;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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

     /*
        // Set the clampRotator to a corresponding state based on if collectingRings is true.
        if (collectingRings.checkState(gamepad2.right_trigger >.2)) {
            robot.ringIntake.extendClampRotator();
        }
        else {
            robot.ringIntake.retractClampRotator();
        }


         */
// clampRotator.setPosition((0.000008513)*intakeArm.getCurrentPosition()*intakeArm.getCurrentPosition()
    //  + (0.0005874)*intakeArm.getCurrentPosition() + 0.2046);

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

        // CWobbleGrabber

    /*
    public void lowerAndUnclamp() {
        clampTimer.reset();

        grabberArm.setPosition(GRABBER_ARM_LOWERED);

        while (clampTimer.milliseconds() < SERVO_ACTIVATION_PAUSE_MS) {}

        wobbleGrabber.setPosition(WOBBLE_GRABBER_UNCLAMPED);
    }

    public void clampAndRaise() {
        clampTimer.reset();

        wobbleGrabber.setPosition(WOBBLE_GRABBER_CLAMPED);

        while (clampTimer.milliseconds() < SERVO_ACTIVATION_PAUSE_MS) {}

        grabberArm.setPosition(GRABBER_ARM_RAISED);
    }
     */

    //CRingShooter
    // Existed to test whether using different speeds for each shooter would result in the shooter
    // shooting diagonally. It didn't.
    /*
    public void ringShoot(double leftSpeed, double rightSpeed) {
        hopperIncrement();

        shooterTimer.reset();

        while (shooterTimer.milliseconds() < HOPPER_RAISE_TIME_MS) {}

        // If the hopper box has raised to a point where a ring is available to shoot,
        // do so.
        if (hopperDepth != 0) {

            leftShooter.setPower(leftSpeed);
            rightShooter.setPower(rightSpeed);

            shooterArm.setPosition(SHOOTER_ARM_ENGAGED);

            shooterTimer.reset();

            while (shooterTimer.milliseconds() < RING_SHOOT_TIME_MS) {}

            setShooterPower(MOTORS_OFF);

            shooterArm.setPosition(SHOOTER_ARM_DISENGAGED);
        }
    }

     */

     /*
    Button              collectingRings = new Button();
    Button              shooterActivated = new Button();
    Button wobbleToggled = new Button();
    boolean             rotatorLocked = false;
    OneShot wobbleButton = new OneShot();
    boolean             armRaised = true;

     */

     /*
        // If the right bumper on Gamepad 2 is pressed, intake a ring (detailed in CRingIntake).
        if(gamepad2.right_bumper) {
            robot.ringIntake.intakeArmTransition = CRingIntake.IntakeArmTransition.DOWN_TO_INTAKE_RING;
        }

        // Set the ringClamp to a corresponding state based on if ringClamped is true..
        if (ringClampToggle.checkState(gamepad2.right_trigger >.2)) {

            ringClamped = !ringClamped;
            if (ringClamped) {
                robot.ringIntake.clampRing();
            } else {
                robot.ringIntake.unclampRing();
            }
        }

        robot.ringIntake.proportionalClampRotator();

        robot.ringIntake.pastIntakeArmPosition = robot.ringIntake.intakeArmPosition;

        if (downToggle.checkState(gamepad2.dpad_down)) {
            robot.ringIntake.intakeArmPosition = CRingIntake.IntakeArmPosition.DOWN;
        }
        else if (hoverToggle.checkState(gamepad2.dpad_right) || hoverToggle.checkState(gamepad2.dpad_left)) {
            robot.ringIntake.intakeArmPosition = CRingIntake.IntakeArmPosition.HOVERING;
        }
        else if (boxToggle.checkState(gamepad2.dpad_up)) {
            robot.ringIntake.intakeArmPosition = CRingIntake.IntakeArmPosition.IN_BOX;
        }

        robot.ringIntake.intakeArmPositionUpdater();

         */

    /*
     public void rotate(double power, double degrees, double slowDegrees) {

        double leftPower, rightPower, differenceAngle, fastDegrees;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        fastDegrees = degrees - slowDegrees;

        if (degrees < 0) {   // turn right.
            leftPower = power;
            rightPower = -power;
        } else if (degrees > 0) {   // turn left.
            leftPower = -power;
            rightPower = power;
        } else return;

        // set power to rotate.
        robot.leftFrontDrive.setPower(leftPower);
        robot.leftBackDrive.setPower(leftPower);
        robot.rightFrontDrive.setPower(rightPower);
        robot.rightBackDrive.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                currentTime = runtime.seconds();
                currentDegrees = getAngle();
                degreeChange = java.lang.Math.abs(degreeDifference(currentDegrees, previousDegrees));
                currentSpeed = degreeChange / currentTime;
                speedChange = (currentSpeed - previousSpeed);
                currentAcceleration = speedChange / currentTime;

                telemetry.addData("Current angle", getAngle());
                telemetry.addData("Loop time", runtime.seconds());
                telemetry.addData("Change in Position (deg)", degreeChange);
                telemetry.addData("Velocity (deg/s)", currentSpeed);
                telemetry.addData("Acceleration (deg/s^2)", currentAcceleration);
                telemetry.update();

                runtime.reset();
                previousDegrees = currentDegrees;
                previousSpeed = currentSpeed;
            }

            while (opModeIsActive() && getAngle() > fastDegrees) {
                currentTime = runtime.seconds();
                currentDegrees = getAngle();
                degreeChange = java.lang.Math.abs(degreeDifference(currentDegrees, previousDegrees));
                currentSpeed = degreeChange / currentTime;
                speedChange = (currentSpeed - previousSpeed);
                currentAcceleration = speedChange / currentTime;

                telemetry.addData("Current angle", getAngle());
                telemetry.addData("Loop time", runtime.seconds());
                telemetry.addData("Change in Position (deg)", degreeChange);
                telemetry.addData("Velocity (deg/s)", currentSpeed);
                telemetry.addData("Acceleration (deg/s^2)", currentAcceleration);
                telemetry.update();

                runtime.reset();
                previousDegrees = currentDegrees;
                previousSpeed = currentSpeed;
            }

            // Set the power super low, and continue.
            leftPower = 0.01;
            rightPower = -0.01;
            robot.leftFrontDrive.setPower(leftPower);
            robot.leftBackDrive.setPower(leftPower);
            robot.rightFrontDrive.setPower(rightPower);
            robot.rightBackDrive.setPower(rightPower);

            while (opModeIsActive() && getAngle() > degrees) {
                currentTime = runtime.seconds();
                currentDegrees = getAngle();
                degreeChange = java.lang.Math.abs(degreeDifference(currentDegrees, previousDegrees));
                currentSpeed = degreeChange / currentTime;
                speedChange = (currentSpeed - previousSpeed);
                currentAcceleration = speedChange / currentTime;

                telemetry.addData("Current angle", getAngle());
                telemetry.addData("Loop time", runtime.seconds());
                telemetry.addData("Change in Position (deg)", degreeChange);
                telemetry.addData("Velocity (deg/s)", currentSpeed);
                telemetry.addData("Acceleration (deg/s^2)", currentAcceleration);
                telemetry.update();

                runtime.reset();
                previousDegrees = currentDegrees;
                previousSpeed = currentSpeed;
            }
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < fastDegrees) {

                currentTime = runtime.seconds();
                currentDegrees = getAngle();
                degreeChange = java.lang.Math.abs(degreeDifference(currentDegrees, previousDegrees));
                currentSpeed = degreeChange / currentTime;
                speedChange = (currentSpeed - previousSpeed);
                currentAcceleration = speedChange / currentTime;

                telemetry.addData("Current angle", getAngle());
                telemetry.addData("Loop time", currentTime);
                telemetry.addData("Change in Position (deg)", degreeChange);
                telemetry.addData("Velocity (deg/ms)", currentSpeed);
                telemetry.addData("Acceleration (deg/ms^2)", currentAcceleration);
                telemetry.update();

                runtime.reset();
                previousDegrees = currentDegrees;
                previousSpeed = currentSpeed;
            }

        // Set the power super low, and continue.
        leftPower = -0.01;
        rightPower = 0.01;
        robot.leftFrontDrive.setPower(leftPower);
        robot.leftBackDrive.setPower(leftPower);
        robot.rightFrontDrive.setPower(rightPower);
        robot.rightBackDrive.setPower(rightPower);

        while (opModeIsActive() && getAngle() < degrees) {

            currentTime = runtime.seconds();
            currentDegrees = getAngle();
            degreeChange = java.lang.Math.abs(degreeDifference(currentDegrees, previousDegrees));
            currentSpeed = degreeChange / currentTime;
            speedChange = (currentSpeed - previousSpeed);
            currentAcceleration = speedChange / currentTime;

            telemetry.addData("Current angle", getAngle());
            telemetry.addData("Loop time", currentTime);
            telemetry.addData("Change in Position (deg)", degreeChange);
            telemetry.addData("Velocity (deg/ms)", currentSpeed);
            telemetry.addData("Acceleration (deg/ms^2)", currentAcceleration);
            telemetry.update();

            runtime.reset();
            previousDegrees = currentDegrees;
            previousSpeed = currentSpeed;
        }

        // turn the motors off.
        robot.leftFrontDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.rightBackDrive.setPower(0);

        sleep(100);
        differenceAngle = degrees - getAngle();

        telemetry.addData("Waiting for the correction", "");
        telemetry.addData("Current angle", getAngle());
        telemetry.addData("Target", degrees);
        telemetry.addData("Difference", differenceAngle);
        telemetry.update();

        // wait for rotation to stop.
        sleep(100);
        // reset angle tracking on new heading.
        resetAngle();
    }
     */

    /*
     /**
     * Resets the cumulative angle tracking to zero.

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle += degreeDifference(angles.firstAngle, lastAngles.firstAngle);

        lastAngles = angles;

        return globalAngle;
    }

    /*

    public void rotate(double power, double degrees) {

        double leftPower, rightPower, differenceAngle;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            leftPower = power;
            rightPower = -power;
        } else if (degrees > 0) {   // turn left.
            leftPower = -power;
            rightPower = power;
        } else return;

        // set power to rotate.
        robot.leftFrontDrive.setPower(leftPower);
        robot.leftBackDrive.setPower(leftPower);
        robot.rightFrontDrive.setPower(rightPower);
        robot.rightBackDrive.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                currentTime = runtime.seconds();
                currentDegrees = getAngle();
                degreeChange = java.lang.Math.abs(degreeDifference(currentDegrees, previousDegrees));
                currentSpeed = degreeChange / currentTime;
                speedChange = (currentSpeed - previousSpeed);
                currentAcceleration = speedChange / currentTime;

                telemetry.addData("Current angle", getAngle());
                telemetry.addData("Loop time", runtime.seconds());
                telemetry.addData("Change in Position (deg)", degreeChange);
                telemetry.addData("Velocity (deg/s)", currentSpeed);
                telemetry.addData("Acceleration (deg/s^2)", currentAcceleration);
                telemetry.update();

                runtime.reset();
                previousDegrees = currentDegrees;
                previousSpeed = currentSpeed;
            }

            while (opModeIsActive() && getAngle() > degrees) {
                currentTime = runtime.seconds();
                currentDegrees = getAngle();
                degreeChange = java.lang.Math.abs(degreeDifference(currentDegrees, previousDegrees));
                currentSpeed = degreeChange / currentTime;
                speedChange = (currentSpeed - previousSpeed);
                currentAcceleration = speedChange / currentTime;

                telemetry.addData("Current angle", getAngle());
                telemetry.addData("Loop time", runtime.seconds());
                telemetry.addData("Change in Position (deg)", degreeChange);
                telemetry.addData("Velocity (deg/s)", currentSpeed);
                telemetry.addData("Acceleration (deg/s^2)", currentAcceleration);
                telemetry.update();

                runtime.reset();
                previousDegrees = currentDegrees;
                previousSpeed = currentSpeed;
            }
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {

                currentTime = runtime.seconds();
                currentDegrees = getAngle();
                degreeChange = java.lang.Math.abs(degreeDifference(currentDegrees, previousDegrees));
                currentSpeed = degreeChange / currentTime;
                speedChange = (currentSpeed - previousSpeed);
                currentAcceleration = speedChange / currentTime;

                telemetry.addData("Current angle", getAngle());
                telemetry.addData("Loop time", currentTime);
                telemetry.addData("Change in Position (deg)", degreeChange);
                telemetry.addData("Velocity (deg/ms)", currentSpeed);
                telemetry.addData("Acceleration (deg/ms^2)", currentAcceleration);
                telemetry.update();

                runtime.reset();
                previousDegrees = currentDegrees;
                previousSpeed = currentSpeed;

            }

        // turn the motors off.
        robot.leftFrontDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.rightBackDrive.setPower(0);

        sleep(100);
        differenceAngle = degrees - getAngle();

        telemetry.addData("Waiting for the correction", "");
        telemetry.addData("Current angle", getAngle());
        telemetry.addData("Target", degrees);
        telemetry.addData("Difference", differenceAngle);
        telemetry.update();

        // wait for rotation to stop.
        sleep(1000);
        // reset angle tracking on new heading.
        resetAngle();
    }
     */

    /*
    private double degreeDifference(double currentAngle, double previousAngle) {

        double deltaAngle = currentAngle - previousAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        return deltaAngle;
    }
     */


        /*

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //moves to line up with the tower shot
                .lineToConstantHeading(new Vector2d(53, -12))
                .build());


        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //second trajectory moves the robot to line up with the second wobble goal
                .lineToLinearHeading(new Pose2d(15, -50, Math.toRadians(-90))) //0
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //moves forward to grab the second wobble goal
                //.forward(14)
                .lineToConstantHeading(new Vector2d(15, -22))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //moves the second wobble goal into box A
                .lineToLinearHeading(new Pose2d(64, 16, Math.toRadians(180)))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //pulls away from the wobble goal and out of the box
                .lineToConstantHeading(new Vector2d(40, 12))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //moves to line up with the power shot targets
                .lineToLinearHeading(new Pose2d(60.5, -27, Math.toRadians(0)))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                // moves onto the white line
                .lineToConstantHeading(new Vector2d(80, -25))
                .build());

         */

    // shooterPosition();
      /*
        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //lines up with the ring goal so it can score and park
                .lineToConstantHeading(new Vector2d(60, -33))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //lines up with the ring goal so it can score and park
                .lineToConstantHeading(new Vector2d(60, -40))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //lines up with the ring goal so it can score and park
                .lineToConstantHeading(new Vector2d(60, -46))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                // .forward(8)
                .lineToConstantHeading(new Vector2d(68, -46))
                .build());

         */


        /*
        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //second trajectory moves the robot into the box
                .lineToConstantHeading(new Vector2d(93, -16))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //pulls back to shoot
                .lineToConstantHeading(new Vector2d(50, -16))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //moves the robot away from the rings
                .lineToConstantHeading(new Vector2d(60, -42))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //second trajectory moves the robot to line up with the second wobble goal
                .lineToLinearHeading(new Pose2d(12, -42, Math.toRadians(-90)))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //moves forward to grab the second wobble goal
                //.forward(14)
                .lineToConstantHeading(new Vector2d(5, 0))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //moves the second wobble goal into box A
                .lineToLinearHeading(new Pose2d(50, 10, Math.toRadians(180)))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //moves into box B
                .lineToConstantHeading(new Vector2d(84, -13))
                .build());

        trajectories.add(robot.drive.trajectoryBuilder(trajectories.get(trajectories.size()-1).end())
                //parks on white line
                .lineToConstantHeading(new Vector2d(70, -5))
                .build());

         */

}
