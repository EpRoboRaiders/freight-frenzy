package org.firstinspires.ftc.teamcode.constructors;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class TensorFlowCamera extends CoreImplement{

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    VuforiaLocalizer.Parameters parameters = null;
    private int tfodMonitorViewId = 0;



    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AYYknDz/////AAABmZQkGhimrULNjspGE2e8mF1m9OCCTsFomWuoUbocgbi0c9q1lDX2CKM/+lRigCqihTfprxW9Ky+zDiXHGZgEvdZqJLUYaKh70oPar3ClwSZH59V/d4cPIYXykkkp4r8YgBXHIxDO+9lvPzqI2pkLFmAD7ysoxfMGJFKSR7HOsz+uB2ChmcRleE5p8rkqkEqTCmZQKensMuT0C4tCcqMFHrJyAHYqxyANLASTqB2vWr5CDQaZP0EX2QyUaeB2NA/41yol6bgzhMApJwlieXYXRT5295LmRHNpkQXOVas/lbfQQaCMZRFwQW6IXQkebzek/bvsMeFTq9hI568MoSc4jJ0igGJpUVQTz/hPZ17MABMi";


    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void init(HardwareMap ahwMap) {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia(ahwMap);
        initTfod(ahwMap);



        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0/9.0);
        }
    }

    @Override
    public void update() {

    }

    private void initVuforia(HardwareMap ahwMap) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = ahwMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod(HardwareMap ahwMap) {
        int tfodMonitorViewId = ahwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", ahwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public int getRingAmount() {
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

        // Has the recognition check worked? If not, default to 0.
        if (updatedRecognitions != null) {

            // Has the recognition seen any rings? If not, run Position A (0 rings).
            if (updatedRecognitions.size() != 0) {

                // The recognizer can only see either 1 or 4 rings; if it doesn't see 4,
                // it determines that it sees 1.
                if (updatedRecognitions.get(0).getLabel() == "Quad") {
                    return 4;
                } else {
                    return 1;
                }
            } else {
                return 0;
            }
        }
        else {
            return 0;
        }

    }
}
