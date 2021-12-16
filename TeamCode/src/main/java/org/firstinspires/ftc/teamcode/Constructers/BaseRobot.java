package org.firstinspires.ftc.teamcode.Constructers;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Cargo.CargoLift;
import org.firstinspires.ftc.teamcode.Constructers.CarouselSpinner.CarouselSpinner;
import org.firstinspires.ftc.teamcode.Constructers.Intake.Intake;
import org.firstinspires.ftc.teamcode.Constructers.Intake.IntakeArm;

public class BaseRobot extends CoreImplement {

    private BaseDriveTrain drivetrain = new BaseDriveTrain();

    private CarouselSpinner spinner = new CarouselSpinner();

    private CargoLift lift = new CargoLift();

    private Intake roller = new Intake();

    private DuckCamera camera = new DuckCamera();


    public static final double DRIVE_SPEED = BaseDriveTrain.DRIVE_SPEED;
    public static final double SLOW_SPEED  = BaseDriveTrain.SLOW_SPEED;

    @Override
    public void init(HardwareMap ahwMap) {
        drivetrain.init(ahwMap);
        spinner.init(ahwMap);
        lift.init(ahwMap);
        roller.init(ahwMap);
        camera.init(ahwMap);

    }

    @Override
    public void update() {
        roller.update();
    }
    public void setLeftBackMotorPower(double power){
        drivetrain.setLeftBackMotorPower(power);
    }
    public void setLeftFrontMotorPower(double power){
        drivetrain.setLeftFrontMotorPower(power);
    }
    public void setRightBackMotorPower(double power){
        drivetrain.setRightBackMotorPower(power);
    }
    public void setRightFrontMotorPower(double power){
        drivetrain.setRightFrontMotorPower(power);
    }
    public void boxMove(double power){
        lift.boxMove(power);
    }
    public void chassisDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS){

        drivetrain.chassisDrive(speed, leftInches, rightInches, timeoutS);
    }

    public void setChassisTankDrivePower(double leftPower, double rightPower){
        drivetrain.setChassisTankDrivePower(leftPower, rightPower);
    }

    /*
     * The following methods are taken directly from the Carousel Spinner subclass, and as such
     * should directly call said methods in their body (so we don't ever have to reference
     * the attachment itself in the "functional code."
     */

    /*
     * Raises the Carousel Spinner Lift, moving it into position for the Carousel Spinner Spinner to
     * spin and deliver Ducks or the Team Shipping Element to the Playing Field Floor.
     */
    public void raiseCarouselSpinnerLift() {
        spinner.raiseCarouselSpinnerLift();
    }

    // Lowers the Carousel Spinner Lift.
    public void lowerCarouselSpinnerLift() {

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
        spinner.startCarouselSpinner();
    }

    // Turns off the Carousel Spinner Spinner.
    public void stopCarouselSpinnerSpinner() {
        spinner.stopCarouselSpinnerSpinner();
    }

    /*
     * Either turns on or off the Carousel Spinner Spinner based on the input variable. Don't just
     * use a boolean for this!
     */
    public void setCarouselSpinnerSpinnerState() {

    }

    public void initialTenInchTest() {
        drivetrain.chassisDrive(0.5, 10, 10, 30);
    }

    public void startRoller(){
        roller.startRoller();
    }

    public void stopRoller(){
        roller.stopRoller();
    }

    public void flick(){
        roller.flick();
    }

    public void reverseRoller(){
        roller.reverseRoller();
    }

    public void secureCargo(){
        lift.secureCargo();
    }

    public void dumpCargo(){
        lift.dumpCargo();
    }

    public void raiseCargoLift(double inches){
        lift.raiseCargoLift(inches);
    }

    public void lowerCargoLift(double inches){
        lift.lowerCargoLift(inches);
    }

    public void directionSwitch(){
        drivetrain.directionSwitch();
    }

    public void resetBox(){
        lift.resetBox();
    }

    public String debug() {
        return drivetrain.debug();
    }

    public String getDuckPosition() {
        return camera.getDuckPosition();
    }

    public void startReverseSpinnerSpinner(){
        spinner.startReverseSpinnerSpinner();
    }
}
