package org.firstinspires.ftc.teamcode;

//import ...


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "TestGyro", group = "GyroTest")


public class GyroTest extends LinearOpMode {

    BNO055IMU imu;
    Orientation currentAngles;
    float previousRaw = 0;
    double previousGlobal = 9999;
    float currentRaw = 0;
    double currentGlobal = 0;
    double globalAngle = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();



        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        currentAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            currentAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentRaw = currentAngles.firstAngle;
            currentGlobal = getAngle();

            if((currentGlobal > previousGlobal) && (previousGlobal !=9999)) {
                telemetry.addData("Left", currentGlobal);
            }
            else
            if((currentGlobal < previousGlobal) && (previousGlobal !=9999)) {
                telemetry.addData("Right", currentGlobal);
            }
            else
            if((currentGlobal == previousGlobal) && (previousGlobal !=9999)) {
                telemetry.addData("Straight", currentGlobal);
            }
            else
            if(previousGlobal != 9999) {
                telemetry.addData("I am lost :(", currentGlobal);
            }
            telemetry.update();

            previousRaw = currentRaw;
            previousGlobal = currentGlobal;
            // might need to remove the sleep when testing is complete
            sleep(500);
        }
    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = currentRaw - previousRaw;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        return globalAngle;
    }

}