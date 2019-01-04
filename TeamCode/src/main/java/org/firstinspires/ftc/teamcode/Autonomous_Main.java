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

import android.graphics.Camera;
import android.hardware.camera2.CameraAccessException;
import android.media.AudioManager;
import android.media.CameraProfile;
import android.media.SoundPool;
import android.text.method.Touch;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.CustomCameraView;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.CustomGoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

@Autonomous(name="Autonomous Main", group="Autonomous")
public class Autonomous_Main extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontleftDrive = null;
    private DcMotor frontrightDrive = null;
    private DcMotor backleftDrive = null;
    private DcMotor backrightDrive = null;

    private DcMotor liftDrive = null;

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

    //Vuforia presetup start
    private static final String VUFORIA_KEY = "Ac+dHab/////AAABme7b8aQeA0UzrrB2riWepIYlMa6BuupJ+4i/AFh4xfuAWQTeCMBvGtwrKST+UBBl4eLykH17nHQMv2akbgoC6F1ztKdMfaUissZAfVCda73PjHVhMovxt99RRIR9EHOEvPJXQYsx7PHtI5hJEtcsdXr3JDXEGZ0bxHQG2rhptuKYb4CNX5b5YO85aY1LFrR/4MqAHEkrmX4rD0TQ2+GVm7/8VKFEEgd7WlcHJbdGz90jk5kxTB3cNbUbn2YuB8BHFYyp+sfHznarsVly7KhmHPviHBGRVpDGUFJ6Q+7/IkLzuEbFytuEKMOluiSEgmXjR/FPDcTBa40Njo/JtO1MjP5v3IBhnRfm3Ti8FGgRgQwn";

    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    VuforiaLocalizer vuforia;

    //Vuforia presetup end

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        frontleftDrive = hardwareMap.get(DcMotor.class, "fl");
        frontrightDrive = hardwareMap.get(DcMotor.class, "fr");
        backleftDrive = hardwareMap.get(DcMotor.class, "bl");
        backrightDrive = hardwareMap.get(DcMotor.class, "br");

        liftDrive = hardwareMap.get(DcMotor.class, "ld");

/*        topLift = hardwareMap.get(TouchSensor.class, "tl");
        lowerLift = hardwareMap.get(TouchSensor.class, "ll");*/

        //Gyro is "imu"

        frontrightDrive.setDirection(DcMotor.Direction.REVERSE);
        backrightDrive.setDirection(DcMotor.Direction.REVERSE);


        frontleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY ;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * This Rover Ruckus sample places a specific target in the middle of each perimeter wall.
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        /**
         * To place the BlueRover target in the middle of the blue perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Then, we translate it along the Y axis to the blue perimeter wall.
         */
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        /**
         * To place the RedFootprint target in the middle of the red perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative Y axis to the red perimeter wall.
         */
        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        /**
         * To place the FrontCraters target in the middle of the front perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative X axis to the front perimeter wall.
         */
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        /**
         * To place the BackSpace target in the middle of the back perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the X axis to the back perimeter wall.
         */
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        /**
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
         * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
         *
         * If using the rear (High Res) camera:
         * We need to rotate the camera around it's long axis to bring the rear camera forward.
         * This requires a negative 90 degree rotation on the Y axis
         *
         * If using the Front (Low Res) camera
         * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
         * This requires a Positive 90 degree rotation on the Y axis
         *
         * Next, translate the camera lens to where it is on the robot.
         * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
         */

        final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        //Dogecv setup_______________________________________________________________________________________________
        detector = new GoldAlignDetector();

        new CustomCameraView(hardwareMap.appContext, 1);

        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
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

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            //--------------------------------------------------------------------------------------
            //Getting off the lander
            //--------------------------------------------------------------------------------------

            //Lower the robot
            liftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            /*liftDrive.setPower(-0.10);

            while (!topLift.isPressed()) Thread.yield();

            liftDrive.setPower(0);*/

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

            } while (detector.getAligned());

            mySound.play(beepID, 1, 1, 1, 0, 1);

            //Hit Block
            double currentEncoderPosition;
            double targetIndividualEncoderPosition = 3880.8;
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