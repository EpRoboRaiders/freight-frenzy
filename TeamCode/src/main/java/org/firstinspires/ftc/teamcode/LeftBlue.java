package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="LeftBlue", group="AutonomousBase")
// @Disabled

public class LeftBlue extends AutonomousBase {
    public void runOpMode() {

        initialize();

        encoderDrive(.1, 12, 12, 12, 12, 3);

        rotate(.1, 90);

        encoderDrive(.1, 12, 12, 12, 12, 3);

        rotate(.1, 90);

        encoderDrive(.1, 12, 12, 12, 12, 3);
    }
}
