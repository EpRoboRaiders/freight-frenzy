package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="AutonomousBase", group="Autonomous")
@Disabled
public abstract class AutonomousBase extends LinearOpMode {

    //test push from home.ee
    RobotTemplate robot = new RobotTemplate();
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 5.2 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.77 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.2;
    static final double     TURN_SPEED              = 0.1;
    double rotation;
    PIDController pidRotate;
    BNO055IMU imu;
    double globalAngle;

    double currentDegrees;
    double previousDegrees;
    double degreeChange;

    double previousSpeed;
    double currentSpeed;
    double speedChange;
    double currentAcceleration;
    double currentTime;

    Orientation lastAngles = new Orientation();

    OpenCvWebcam webcam;
    RingStackMeasurerPipeline pipeline;

    public void runOpMode() {}

    public void encoderDrive(double speed, double leftFrontInches, double leftBackInches,
                             double rightFrontInches, double rightBackInches, double timeoutS) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBackDrive.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);

            robot.leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            robot.leftBackDrive.setTargetPosition(newLeftBackTarget);
            robot.rightFrontDrive.setTargetPosition(newRightFrontTarget);
            robot.rightBackDrive.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFrontDrive.setPower(Math.abs(speed));
            robot.leftBackDrive.setPower(Math.abs(speed));
            robot.rightFrontDrive.setPower(Math.abs(speed));
            robot.rightBackDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontDrive.isBusy() && robot.leftBackDrive.isBusy()
                            && robot.rightFrontDrive.isBusy() && robot.rightBackDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d %7d %7d", newLeftFrontTarget, newLeftBackTarget, newRightFrontTarget, newRightBackTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d %7d %7d",
                        robot.leftFrontDrive.getCurrentPosition(),
                        robot.leftBackDrive.getCurrentPosition(),
                        robot.rightFrontDrive.getCurrentPosition(),
                        robot.rightBackDrive.getCurrentPosition()
                );
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFrontDrive.setPower(0);
            robot.leftBackDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
            robot.rightBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(100);   // optional pause after each move
        }
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
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
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
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




    public static class RingStackMeasurerPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(181,98);

        static final int REGION_WIDTH = 70; // 35
        static final int REGION_HEIGHT = 50; // 25

        final int FOUR_RING_THRESHOLD = 140;
        final int ONE_RING_THRESHOLD = 130;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile RingStackMeasurerPipeline.RingPosition position = RingStackMeasurerPipeline.RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingStackMeasurerPipeline.RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingStackMeasurerPipeline.RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingStackMeasurerPipeline.RingPosition.ONE;
            }else{
                position = RingStackMeasurerPipeline.RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }

    public void initialize() {

        robot.init(hardwareMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate = new PIDController(.09, .0009, 0);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new RingStackMeasurerPipeline();
        webcam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        //webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
        });
        telemetry.addData("Status", "Camera initializing; please wait (~10 seconds)");
        telemetry.update();
        sleep(10000);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        waitForStart();
        robot.wobbleGrabber.setPower(.05);
    }
    private double degreeDifference(double currentAngle, double previousAngle) {

        double deltaAngle = currentAngle - previousAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        return deltaAngle;
    }
}
