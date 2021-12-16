package org.firstinspires.ftc.teamcode.Constructers.CarouselSpinner;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constructers.CoreImplement;

 /*
  * The Carousel Spinner Spinner is a CR Servo that spins to rotate the Carousel and deliver ducks
  * or the Team Shipping Element to the Playing Field Floor.
  */
public class CarouselSpinnerSpinner extends CoreImplement {

    public CRServo spinnerServo;

    // Initializes the Carousel Spinner Spinner.
    @Override
    public void init(HardwareMap ahwMap) {
        spinnerServo  = ahwMap.get(CRServo.class, "carousel_spinner");
    }

    // "Updates" the Carousel Spinner Spinner. You can probably leave this one blank for now.
    @Override
    public void update() {

    }

    /*
     * Turns on the Carousel Spinner Spinner, to spin the Carousel and deliver ducks or the Team
     * Shipping Element to the Playing Field Floor.
     */
    public void startCarouselSpinnerSpinner() {
        spinnerServo.setPower(0.5);
    }

    // Turns off the Carousel Spinner Spinner.
    public void stopCarouselSpinnerSpinner() {
        spinnerServo.setPower(0);
    }

    public void startReverseSpinnerSpinner(){
        spinnerServo.setPower(-0.5);
    }

     /*
      * Either turns on or off the Carousel Spinner Spinner based on the input variable. Don't just
      * use a boolean for this!
      */
    //public void setCarouselSpinnerSpinnerState(CarouselSpinnerSpinnerStates state) {

    //}
}
