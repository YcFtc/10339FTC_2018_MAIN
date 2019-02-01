package org.firstinspires.ftc.teamcode;

import android.media.AudioManager;
import android.media.SoundPool;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@Autonomous(name="Autonomous Depot", group="Linear Opmode")
public class Autonomous_Depot extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontleftDrive = null;
    private DcMotor frontrightDrive = null;

    private DcMotor backleftDrive = null;
    private DcMotor backrightDrive = null;

    private DcMotor slideMotor = null;

    private Servo knocker = null;

    private TouchSensor topLift = null;
    private TouchSensor lowerLift = null;

    private TouchSensor linearSlideOut = null;
    private TouchSensor linearSlideIn = null;

    //Detector object
    private GoldAlignDetector detector;

    double dogecvCameraCenter = 210;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    private SoundPool mySound;
    private int beepID;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        frontleftDrive = hardwareMap.get(DcMotor.class, "fl");
        frontrightDrive = hardwareMap.get(DcMotor.class, "fr");

        backleftDrive = hardwareMap.get(DcMotor.class, "bl");
        backrightDrive = hardwareMap.get(DcMotor.class, "br");

        slideMotor = hardwareMap.get(DcMotor.class, "sm");

        knocker = hardwareMap.get(Servo.class, "k");

        linearSlideOut = hardwareMap.get(TouchSensor.class, "lso");
        linearSlideIn = hardwareMap.get(TouchSensor.class, "lsi");

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

        //Dogecv setup_______________________________________________________________________________________________
        detector = new GoldAlignDetector();

        detector.init(hardwareMap.appContext,CameraViewDisplay.getInstance(), 0, false);
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;//was 5
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();

        mySound = new SoundPool(1, AudioManager.STREAM_MUSIC, 0); // PSM
        beepID = mySound.load(hardwareMap.appContext, R.raw.beep, 10); // PSM
        // To play \/
        // mySound.play(beepID, 1, 1, 1, 0, 1);

        waitForStart();
        runtime.reset();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 250);

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            //--------------------------------------------------------------------------------------
            //Getting off the lander
            //--------------------------------------------------------------------------------------

            //5.5 motor turns until lift extended
            //Neverest 40 has 1120 encoder ticks per output shaft revolution

            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //1120 * 5.5 wheel turns
            slideMotor.setTargetPosition(6160);

            slideMotor.setPower(1);

            while (slideMotor.isBusy()) Thread.yield();

            frontleftDrive.setPower(-0.3);
            frontrightDrive.setPower(-0.3);

            backleftDrive.setPower(-0.3);
            backrightDrive.setPower(-0.3);

            sleep(300);

            frontleftDrive.setPower(0);
            frontrightDrive.setPower(0);

            backleftDrive.setPower(0);
            backrightDrive.setPower(0);


            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            slideMotor.setPower(-1);

            while (!linearSlideIn.isPressed()) Thread.yield();

            slideMotor.setPower(0);


            frontleftDrive.setPower(0.3);
            frontrightDrive.setPower(0.3);

            backleftDrive.setPower(0.3);
            backrightDrive.setPower(0.3);

            sleep(300);

            frontleftDrive.setPower(0);
            frontrightDrive.setPower(0);

            backleftDrive.setPower(0);
            backrightDrive.setPower(0);

            //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            //Sampling
            //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

            frontleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            backleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            telemetry.addData("Waiting to turn", detector.isFound());
            telemetry.update();

            while (!detector.isFound()) Thread.yield();

            //Aligning with Gold
            do {
                if (detector.getXPosition() > dogecvCameraCenter) {
                    telemetry.addData("Is Left", detector.getXPosition());
                    telemetry.update();

                    RUN_DRIVE_MANUAL(-0.15, 0.15);

                } else if (detector.getXPosition() < dogecvCameraCenter) {
                    telemetry.addData("Is Right", detector.getXPosition());
                    telemetry.update();

                    RUN_DRIVE_MANUAL(0.15, -0.15);
                }

/*                sleep(100);
                RUN_DRIVE_MANUAL(0, 0);*/
                telemetry.addData("Getting Aligned", detector.getAligned());
                telemetry.update();
//                sleep(300);

                if (detector.getAligned()) break;

                telemetry.addData("Not Aligned", detector.getAligned());
                telemetry.update();

            } while (true);

            RUN_DRIVE(0, 0);

            mySound.play(beepID, 1, 1, 1, 0, 1);

            telemetry.addData(" Waiting to Hit Block", detector.getAligned());
            telemetry.update();

            sleep(1000);

            telemetry.addData("Hitting Block", detector.getAligned());
            telemetry.update();


            STOP_AND_RESET_DRIVE_ENCODERS();

            frontleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            backleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //Hit Block
            int currentEncoderPosition;
            int targetIndividualEncoderPosition = 537 * 3; //537 Encoder ticks per turn multiplied by 3 turns
            int targetEncoderPosition = targetIndividualEncoderPosition * 4; //3.5 wheel turns multiplied by 4 for all motors

            /*do {
                //powers were 0.75 or 0.70
                if (detector.isFound()) RUN_DRIVE_STRAFE(-0.1, -0.1);
                else if (detector.getXPosition() > dogecvCameraCenter*//*to the left*//*)
                    RUN_DRIVE_STRAFE(-0.05, -0.1);
                else if (detector.getXPosition() < dogecvCameraCenter*//*to the right*//*)
                    RUN_DRIVE_STRAFE(-0.1, -0.05);


                currentEncoderPosition = frontleftDrive.getCurrentPosition() +
                        frontrightDrive.getCurrentPosition() +
                        backleftDrive.getCurrentPosition() +
                        backrightDrive.getCurrentPosition();

                if (frontleftDrive.getCurrentPosition() >= targetIndividualEncoderPosition)
                    frontleftDrive.setPower(0);
                if (frontrightDrive.getCurrentPosition() >= targetIndividualEncoderPosition)
                    frontrightDrive.setPower(0);

                if (backleftDrive.getCurrentPosition() >= targetIndividualEncoderPosition)
                    backleftDrive.setPower(0);
                if (backrightDrive.getCurrentPosition() >= targetIndividualEncoderPosition)
                    backrightDrive.setPower(0);

            } while (!(currentEncoderPosition >= targetEncoderPosition));*/

            RUN_DRIVE_STRAFE(-0.1, -0.1);

            sleep(3500);

            RUN_DRIVE(0, 0);

            detector.disable();

            telemetry.addData("Hitted", "yes");
            telemetry.update();

/*
            RUN_TO_POSITION();

            RUN_DRIVE(-0.5, 0);

            RUN_DRIVE_MANUAL(0, 0);
*/

            telemetry.addData("Done", "Done");
            telemetry.update();

            //======================================================================================
            //Depot
            //======================================================================================

            telemetry.clearAll();

            // Set up our telemetry dashboard
            composeTelemetry();

            //Angles order is ZYX
            //IMU left is bigger

            do {
                telemetry.update();

                if (angles.firstAngle < -10) RUN_DRIVE_MANUAL(0.1, -0.1);
                else if (angles.firstAngle > 10) RUN_DRIVE_MANUAL(-0.1, 0.1);

            } while (angles.firstAngle < -10 && angles.firstAngle > 10);

            RUN_DRIVE_MANUAL(0, 0);

            sleep(200);

            int encoderStart = frontleftDrive.getCurrentPosition();

            do {
                telemetry.update();

                if (angles.firstAngle < -5) RUN_DRIVE_MANUAL(0.3, 0.25);
                else if (angles.firstAngle > 5) RUN_DRIVE_MANUAL(0.25, 0.3);
                else RUN_DRIVE_MANUAL(0.3, 0.3);

            } while ((frontleftDrive.getCurrentPosition() - encoderStart) <= (537/*encoder value*/ * 2));

            sleep(200);

            knocker.setPosition(1);

            sleep(750);

            knocker.setPosition(0);


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
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

    private void RUN_DRIVE(double ForwardPower, int ForwardWheelTurns){

        if (ForwardWheelTurns != 0){
//            int ticks_to_turn = (int) Math.rint(ForwardWheelTurns * 537);/*Rounding WheelTurns * 537 to the closest integer value and then casting/converting it to an int*/


            int ticks_to_turn = ForwardWheelTurns * 537;

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

    private void RUN_DRIVE_STRAFE(double FrontRightwardPower, double BackRightwardPower) {
        frontleftDrive.setPower(-FrontRightwardPower);
        frontrightDrive.setPower(FrontRightwardPower);

        backleftDrive.setPower(BackRightwardPower);
        backrightDrive.setPower(-BackRightwardPower);
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

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

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
