package org.firstinspires.ftc.teamcode;

import android.media.AudioManager;
import android.media.SoundPool;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Autonomous Main", group="Autonomous")
public class Autonomous_Main extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontleftDrive = null;
    private DcMotor frontrightDrive = null;

    private DcMotor backleftDrive = null;
    private DcMotor backrightDrive = null;

    private DcMotor slideMotor = null;

    private TouchSensor topLift = null;
    private TouchSensor lowerLift = null;

    private TouchSensor linearSlideOut = null;
    private TouchSensor linearSlideIn = null;

    //Detector object
    private GoldAlignDetector detector;

    double dogecvCameraCenter = 210;

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

            sleep(3500);

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

            //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            //Sampling Portion
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
}
