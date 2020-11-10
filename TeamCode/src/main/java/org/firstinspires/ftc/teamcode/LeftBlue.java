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
            // Placement A
            positionA();
        } else if (pipeline.position == RingStackMeasurerPipeline.RingPosition.ONE) {
            // Placement B
            positionB();

        } else /* if(pipeline.position == RingStackMeasurerPipeline.RingPosition.FOUR) */ {
            // Placement C (Default)
            positionC();
        }
    }

    public void positionA() {
        // Wobble Goal Placement A
        encoderDrive(.1, 60, 60, 60, 60, 5);

        rotate(.1, 45);

        encoderDrive(.1, 8, 8, 8, 8, 5);

        encoderDrive(.1, -19, -19, -19, -19, 5);

        rotate(.1, -45);

        encoderDrive(.1, 20, 20, 20, 20, 5);
    }

    public void positionB() {
        // Wobble Goal Placement B
        encoderDrive(.1, 88, 88, 88, 88, 5);

        rotate(.2, -45);

        encoderDrive(.1, 10,10,10,10,5);

        encoderDrive(.1, -16, -16, -16, -16, 5);
    }

    public void positionC() {
        // Wobble Goal Placement C
        encoderDrive(.1, 109, 109, 109, 109, 10);

        rotate(.1, 45);

        encoderDrive(.1, 8, 8, 8, 8, 5);

        encoderDrive(.1, -18, -18, -18, -18, 5);

        rotate(.1, -45);

        encoderDrive(.1, -30, -30, -30, -30, 5);
    }
}

