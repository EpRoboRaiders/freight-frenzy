package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constructers.BaseRobot;

@Autonomous(name = "LiftTest", group = "Autonomous")
public class LiftTest extends LinearOpMode {

    private BaseRobot robot = new BaseRobot();

    /**
     * Override this method and place your code here.
     * <p>
     * Please do not swallow the InterruptedException, as it is used in cases
     * where the op mode needs to be terminated early.
     *
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();
       double duckposition = robot.getDuckPosition();
       telemetry.addData("duckposition : ",duckposition);
       telemetry.update();
        robot.raiseCargoLift(duckposition);

        sleep(999999999);
    }
}
