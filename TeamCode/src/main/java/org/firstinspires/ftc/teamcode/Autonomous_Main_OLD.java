package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import android.media.AudioManager;
import android.media.SoundPool;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.CustomCameraView;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigationWebcam;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name="Autonomous Main", group="Autonomous")
@Disabled
public class Autonomous_Main_OLD extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontleftDrive = null;
    private DcMotor frontrightDrive = null;
    private DcMotor backleftDrive = null;
    private DcMotor backrightDrive = null;

    private DcMotor slideMotor = null;

    private TouchSensor topLift = null;
    private TouchSensor lowerLift = null;

    private GoldAlignDetector detector;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    double dogecvCameraCenter = 210;

    private SoundPool mySound;
    private int beepID;

/*
    //Vuforia presetup start
    private static final String VUFORIA_KEY = "Ac+dHab/////AAABme7b8aQeA0UzrrB2riWepIYlMa6BuupJ+4i/AFh4xfuAWQTeCMBvGtwrKST+UBBl4eLykH17nHQMv2akbgoC6F1ztKdMfaUissZAfVCda73PjHVhMovxt99RRIR9EHOEvPJXQYsx7PHtI5hJEtcsdXr3JDXEGZ0bxHQG2rhptuKYb4CNX5b5YO85aY1LFrR/4MqAHEkrmX4rD0TQ2+GVm7/8VKFEEgd7WlcHJbdGz90jk5kxTB3cNbUbn2YuB8BHFYyp+sfHznarsVly7KhmHPviHBGRVpDGUFJ6Q+7/IkLzuEbFytuEKMOluiSEgmXjR/FPDcTBa40Njo/JtO1MjP5v3IBhnRfm3Ti8FGgRgQwn";

    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor
*/

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

//    Dogeforia vuforia;

    WebcamName webcamName;

    //Vuforia presetup end

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        frontleftDrive = hardwareMap.get(DcMotor.class, "fl");
        frontrightDrive = hardwareMap.get(DcMotor.class, "fr");
        backleftDrive = hardwareMap.get(DcMotor.class, "bl");
        backrightDrive = hardwareMap.get(DcMotor.class, "br");

        slideMotor = hardwareMap.get(DcMotor.class, "sm");

/*        topLift = hardwareMap.get(TouchSensor.class, "tl");
        lowerLift = hardwareMap.get(TouchSensor.class, "ll");*/

        //Gyro is "imu"

        frontrightDrive.setDirection(DcMotor.Direction.REVERSE);
        backrightDrive.setDirection(DcMotor.Direction.REVERSE);


        frontleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //GYRO setup_________________________________________________________________________________________________

        // Set up the parameters1 with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
        parameters1.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters1.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters1.loggingEnabled      = true;
        parameters1.loggingTag          = "IMU";
        parameters1.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters1.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters1);

        //Vuforia setup______________________________________________________________________________________________
//        /*
//         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
//         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
//         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
//         */
//
//        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
//
//        /*
//         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
//         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
//         */
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        Dogeforia.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//
//        // OR...  Do Not Activate the Camera Monitor View, to save power
//        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//
//        /*
//         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
//         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
//         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
//         * web site at https://developer.vuforia.com/license-manager.
//         *
//         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
//         * random data. As an example, here is a example of a fragment of a valid key:
//         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
//         * Once you've obtained a license key, copy the string from the Vuforia web site
//         * and paste it in to your code on the next line, between the double quotes.
//         */
//        parameters.vuforiaLicenseKey = "Ac+dHab/////AAABme7b8aQeA0UzrrB2riWepIYlMa6BuupJ+4i/AFh4xfuAWQTeCMBvGtwrKST+UBBl4eLykH17nHQMv2akbgoC6F1ztKdMfaUissZAfVCda73PjHVhMovxt99RRIR9EHOEvPJXQYsx7PHtI5hJEtcsdXr3JDXEGZ0bxHQG2rhptuKYb4CNX5b5YO85aY1LFrR/4MqAHEkrmX4rD0TQ2+GVm7/8VKFEEgd7WlcHJbdGz90jk5kxTB3cNbUbn2YuB8BHFYyp+sfHznarsVly7KhmHPviHBGRVpDGUFJ6Q+7/IkLzuEbFytuEKMOluiSEgmXjR/FPDcTBa40Njo/JtO1MjP5v3IBhnRfm3Ti8FGgRgQwn";
//
//
//        /**
//         * We also indicate which camera on the RC we wish to use. For pedagogical purposes,
//         * we use the same logic as in {@link ConceptVuforiaNavigationWebcam}.
//         */
//        parameters.cameraName = webcamName;
//        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//        /**
//         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
//         * in this data set: all three of the VuMarks in the game were created from this one template,
//         * but differ in their instance id information.
//         * @see VuMarkInstanceId
//         */
//        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
//        VuforiaTrackable relicTemplate = relicTrackables.get(0);
//        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        //Dogecv setup_______________________________________________________________________________________________
        detector = new GoldAlignDetector();

        new CustomCameraView(hardwareMap.appContext, 1);

        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 0, true);
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();

        mySound = new SoundPool(1, AudioManager.STREAM_MUSIC, 0); // PSM
        beepID = mySound.load(hardwareMap.appContext, R.raw.beep, 10); // PSM
        // To play \/
        // mySound.play(beepID, 1, 1, 1, 0, 1);


        // Set up our telemetry dashboard
        composeTelemetry();

//        vuforia.setDogeCVDetector(detector);
//        vuforia.enableDogeCV();
//        vuforia.showDebug();
//        vuforia.start();
        
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            //--------------------------------------------------------------------------------------
            //Getting off the lander
            //--------------------------------------------------------------------------------------

            //Lower the robot
            slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            slideMotor.setPower(-0.1);

            sleep(10000);

            slideMotor.setPower(0);

            /*slideMotor.setPower(-0.10);

            while (!topLift.isPressed()) Thread.yield();

            slideMotor.setPower(0);*/

            //change msPollInterval if phone is lagging too much
            imu.startAccelerationIntegration(new Position(), new Velocity(), 200);

            RUN_DRIVE_MANUAL(0.5, -0.5);

            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            while (angles.firstAngle < -90) {
                composeTelemetry();
                telemetry.update();
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                sleep(100);
            }

            imu.stopAccelerationIntegration();
            RUN_DRIVE_MANUAL(0, 0);



            //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            //Sampling Portion
            //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

            STOP_AND_RESET_DRIVE_ENCODERS();

            //Aligning with Gold
            do {
                if (detector.getXPosition() < dogecvCameraCenter + 10) {
                    telemetry.addData("Is Left", detector.getXPosition());
                    telemetry.update();

                    RUN_DRIVE_MANUAL(-0.15, 0.15);

                    while (detector.getXPosition() < dogecvCameraCenter + 10) Thread.yield();
                } else if (detector.getXPosition() > dogecvCameraCenter - 10) {
                    telemetry.addData("Is Right", detector.getXPosition());
                    telemetry.update();

                    RUN_DRIVE_MANUAL(0.15, -0.15);

                    while (detector.getXPosition() > dogecvCameraCenter - 10) Thread.yield();
                }
                RUN_DRIVE_MANUAL(0, 0);

                sleep(1500);

            } while (!detector.getAligned());

            mySound.play(beepID, 1, 1, 1, 0, 1);

            //Hit Block
            double currentEncoderPosition;
            double targetIndividualEncoderPosition = 806.4;
            double targetEncoderPosition = targetIndividualEncoderPosition * 4; //3.5 wheel turns multiplied by 0.75 for demobot and 4 for all motors
            STOP_AND_RESET_DRIVE_ENCODERS();
            do {
                currentEncoderPosition = frontleftDrive.getCurrentPosition() +
                        frontrightDrive.getCurrentPosition() +
                        backleftDrive.getCurrentPosition() +
                        backrightDrive.getCurrentPosition();

                if (detector.isFound() || detector.getAligned()) RUN_DRIVE_MANUAL(0.75, 0.75);
                else if (detector.getXPosition() < dogecvCameraCenter/*to the left*/)
                    RUN_DRIVE_MANUAL(0.70, 0.75);
                else if (detector.getXPosition() > dogecvCameraCenter/*to the right*/)
                    RUN_DRIVE_MANUAL(0.75, 0.70);

                if (frontleftDrive.getCurrentPosition() >= targetIndividualEncoderPosition)
                    frontleftDrive.setPower(0);
                if (frontrightDrive.getCurrentPosition() >= targetIndividualEncoderPosition)
                    frontrightDrive.setPower(0);

                if (backleftDrive.getCurrentPosition() >= targetIndividualEncoderPosition)
                    backleftDrive.setPower(0);
                if (backrightDrive.getCurrentPosition() >= targetIndividualEncoderPosition)
                    backrightDrive.setPower(0);

            } while (!(currentEncoderPosition >= targetEncoderPosition));
            RUN_DRIVE(0, 0);

            detector.disable();

            RUN_TO_POSITION();

            RUN_DRIVE(-0.5, 0);


            //======================================================================================
            //Depo Portion
            //======================================================================================

            //Turn to VuMark
            imu.startAccelerationIntegration(new Position(), new Velocity(), 200);

            RUN_DRIVE_MANUAL(0.2, -0.2);

            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            while (angles.firstAngle < -45) {
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                sleep(100);
            }

            imu.stopAccelerationIntegration();
            RUN_DRIVE_MANUAL(0, 0);

            stop();
        }
    }

    private void RUN_DRIVE_MANUAL(double leftPower, double rightPower) {
        leftPower = Range.clip(leftPower, -1, 1);
        rightPower = Range.clip(rightPower, -1, 1);

        frontleftDrive.setPower(leftPower);
        frontrightDrive.setPower(rightPower);
        backleftDrive.setPower(leftPower);
        backrightDrive.setPower(rightPower);
    }

    private void RUN_DRIVE(double ForwardPower, double ForwardWheelTurns){

        if (ForwardWheelTurns != 0){
//            int ticks_to_turn = (int) Math.rint(ForwardWheelTurns * 1478.4);/*Rounding WheelTurns * 1478.4 to the closest integer value and then casting/converting it to an int*/

//           TEMP FOR DEMOBOT
            int ticks_to_turn = (int) Math.rint((ForwardWheelTurns * 1478.4) * 0.75);

            frontleftDrive.setTargetPosition(ticks_to_turn);
            frontrightDrive.setTargetPosition(ticks_to_turn);

            backleftDrive.setTargetPosition(ticks_to_turn);
            backrightDrive.setTargetPosition(ticks_to_turn);
        }

        frontleftDrive.setPower(ForwardPower);
        frontrightDrive.setPower(ForwardPower);

        backleftDrive.setPower(ForwardPower);
        backrightDrive.setPower(ForwardPower);
    }

    private void STOP_AND_RESET_DRIVE_ENCODERS(){
        frontleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void RUN_USING_ENCODER_ALL_DRIVE(){
        frontleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void RUN_WITHOUT_ENCODER_ALL_DRIVE(){
        frontleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void RUN_TO_POSITION() {
        frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void WAITFORDRIVE (){
        while (frontleftDrive.isBusy() || frontrightDrive.isBusy() || backleftDrive.isBusy() || backrightDrive.isBusy()){
            idle();
        }
    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}