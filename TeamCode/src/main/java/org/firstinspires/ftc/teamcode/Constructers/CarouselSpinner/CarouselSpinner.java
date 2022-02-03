package org.firstinspires.ftc.teamcode.Constructers.CarouselSpinner;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constructers.CoreImplement;

/*
 * The Carousel Spinner is a attachment, in which a Carousel Spinner Lift raises a Carousel Spinner
 * Spinner into position to rotate the Carousel and deliver ducks or the Team Shipping Element to
 * the Playing Field Floor.
 */
public class CarouselSpinner extends CoreImplement {

    private CarouselSpinnerSpinner spinner = new CarouselSpinnerSpinner();
    private CarouselSpinnerLift spinnerLift = new CarouselSpinnerLift();

    // Initializes the Carousel Spinner.
    @Override
    public void init(HardwareMap ahwMap) {
        spinner.init(ahwMap);
    }

    // "Updates" the Carousel Spinner. You can probably leave this one blank for now.
    @Override
    public void update() {

    }

    /*
     * The following methods are taken directly from the Carousel Spinner [n] subclasses, and as
     * such should directly call said methods in their body (so we don't ever have to reference
     * the attachments themselves in the "functional code."
     */

    /*
     * Raises the Carousel Spinner Lift, moving it into position for the Carousel Spinner Spinner to
     * spin and deliver Ducks or the Team Shipping Element to the Playing Field Floor.
     */
    public void raiseCarouselSpinnerLift() {
        spinnerLift.raiseCarouselSpinnerLift();

    }

    // Lowers the Carousel Spinner Lift.
    public void lowerCarouselSpinnerLift() {
        spinnerLift.lowerCarouselSpinnerLift();
    }

    /*
     * Either raises or lowers the Carousel Spinner Lift based on the input variable. Don't just use
     * a boolean for this!
     */
    public void setCarouselSpinnerLiftState() {

    }

    /*
     * Turns on the Carousel Spinner Spinner, to spin the Carousel and deliver ducks or the Team
     * Shipping Element to the Playing Field Floor.
     */
    public void startCarouselSpinner() {
        spinner.startCarouselSpinnerSpinner();
    }

    // Turns off the Carousel Spinner Spin  ner.
    public void stopCarouselSpinnerSpinner() {
        spinner.stopCarouselSpinnerSpinner();
    }

    public void startReverseSpinnerSpinner(){
        spinner.startReverseSpinnerSpinner();
    }

    /*
     * Either turns on or off the Carousel Spinner Spinner based on the input variable. Don't just
     * use a boolean for this!
     */
    public void setCarouselSpinnerSpinnerState() {

    }
}
