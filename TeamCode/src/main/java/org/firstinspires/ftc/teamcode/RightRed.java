package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Turning Test", group="AutonomousBase")
public class RightRed extends AutonomousBase {

    @Override
    public void runOpMode() {

        initialize();


        rotate(.1, 45);

        sleep(3000);

        rotate(.1, 90);
    }
}

