package org.firstinspires.ftc.teamcode.Constructers.CarouselSpinner;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constructers.CoreImplement;

/*
 * The Carousel Spinner Spinner is a 180 degree Servo that raises and lowers the Carousel Spinner
 * Spinner, which in turn spins to rotate the Carousel and deliver ducks
 * or the Team Shipping Element to the Playing Field Floor.
 */
public class CarouselSpinnerLift extends CoreImplement {

        private DcMotor liftmotor;

    // Initializes the Carousel Spinner Lift.
    @Override
    public void init(HardwareMap ahwMap) {
        liftmotor = ahwMap.get(DcMotor.class, "liftmotor");
        liftmotor.setPower(0);
        liftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftmotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors

    }

    // "Updates" the Carousel Spinner Lift. You can probably leave this one blank for now.
    @Override
    public void update() {

    }

    /*
     * Raises the Carousel Spinner Lift, moving it into position for the Carousel Spinner Spinner to
     * spin and deliver Ducks or the Team Shipping Element to the Playing Field Floor.
     */
    public void raiseCarouselSpinnerLift() {
        liftmotor.setPower(0.5);
        sleep(500);
        liftmotor.setPower(0);
    }

    // Lowers the Carousel Spinner Lift.
    public void lowerCarouselSpinnerLift() {
        liftmotor.setPower(-0.5);
        sleep(500);
        liftmotor.setPower(0);
    }

    /*
     * Either raises or lowers the Carousel Spinner Lift based on the input variable. Don't just use
     * a boolean for this!
     */
    //public void setCarouselSpinnerLiftState(CarouselSpinnerLiftStates state) {


    //}
}
