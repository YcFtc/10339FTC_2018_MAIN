package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.opencv.core.Mat;

@TeleOp(name="Mainbot Code", group="Linear Opmode")
public class Mainbot_Code extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontleftDrive = null;
    private DcMotor frontrightDrive = null;
    private DcMotor backleftDrive = null;
    private DcMotor backrightDrive = null;

//    private Servo RGB = null;

//    private TouchSensor topTouch = null;
//    private TouchSensor lowerTouch = null;

    private DcMotor linearMotor = null;

//    private TouchSensor linearSlideOut = null;
//    private TouchSensor linearSlideIn = null;

    private DcMotor linearLift = null;
    
    private Servo leftIntake = null;
    private Servo rightIntake = null;

    private Servo linearStop = null;
    private Servo liftStop = null;

    double linearPower = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        frontleftDrive = hardwareMap.get(DcMotor.class, "fl");
        frontrightDrive = hardwareMap.get(DcMotor.class, "fr");
        backleftDrive = hardwareMap.get(DcMotor.class, "bl");
        backrightDrive = hardwareMap.get(DcMotor.class, "br");

//        RGB = hardwareMap.get(Servo.class, "rgb");

//        topTouch = hardwareMap.get(TouchSensor.class, "tt");
//        lowerTouch = hardwareMap.get(TouchSensor.class, "lt");

        linearMotor = hardwareMap.get(DcMotor.class, "lm");

//        linearSlideOut = hardwareMap.get(TouchSensor.class, "lso");
//        linearSlideIn = hardwareMap.get(TouchSensor.class, "lsi");

        linearLift = hardwareMap.get(DcMotor.class, "ll");
        
        leftIntake = hardwareMap.get(Servo.class, "li");
        rightIntake = hardwareMap.get(Servo.class, "ri");

        linearStop = hardwareMap.get(Servo.class, "linearStop");

        liftStop = hardwareMap.get(Servo.class, "liftStop");

        //Gyro is "imu"

        frontrightDrive.setDirection(DcMotor.Direction.REVERSE);
        backrightDrive.setDirection(DcMotor.Direction.REVERSE);


        frontleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        backleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        linearLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        linearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double frontleftDrivePower;
        double frontrightDrivePower;

        double backleftDrivePower;
        double backrightDrivePower;

        linearStop.setPosition(0);
        liftStop.setPosition(0);

//        double RGB_status = 0;

        waitForStart();
        runtime.reset();

        frontleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double linearLiftMax = linearLift.getCurrentPosition() + 147;

        double linearLiftPower = 0;

        linearLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {
            frontleftDrivePower = -gamepad1.left_stick_y /*For driving forward/backward*/ + gamepad1.right_stick_x /*Turning*/ + gamepad1.left_stick_x /*Strafing*/;
            frontrightDrivePower = -gamepad1.left_stick_y /*For driving forward/backward*/ - gamepad1.right_stick_x /*Turning*/ - gamepad1.left_stick_x /*Strafing*/;

            backleftDrivePower = -gamepad1.left_stick_y /*For driving forward/backward*/ + gamepad1.right_stick_x /*Turning*/ - gamepad1.left_stick_x /*Strafing*/;
            backrightDrivePower = -gamepad1.left_stick_y /*For driving forward/backward*/ - gamepad1.right_stick_x /*Turning*/ + gamepad1.left_stick_x /*Strafing*/;

            frontleftDrivePower = Range.clip(frontleftDrivePower * Math.abs(frontleftDrivePower), -1, 1);
            frontrightDrivePower = Range.clip(frontrightDrivePower * Math.abs(frontrightDrivePower), -1, 1);

            backleftDrivePower = Range.clip(backleftDrivePower * Math.abs(backleftDrivePower), -1, 1);
            backrightDrivePower = Range.clip(backrightDrivePower * Math.abs(backrightDrivePower), -1, 1);

            frontleftDrive.setPower(frontleftDrivePower);
            frontrightDrive.setPower(frontrightDrivePower);

            backleftDrive.setPower(backleftDrivePower);
            backrightDrive.setPower(backrightDrivePower);

            linearLiftPower = -gamepad2.right_stick_y;

//            if (linearLift.getCurrentPosition() <= linearLiftMax && linearLiftPower < 0) linearLiftPower = 0;

            linearLift.setPower(linearLiftPower);

            linearPower = gamepad2.left_stick_y;

//            if (linearSlideIn.isPressed() && linearPower < 0) linearPower = 0;
//            else if (linearSlideOut.isPressed() && linearPower > 0) linearPower = 0;

            linearMotor.setPower(linearPower);

            if (gamepad2.left_bumper) leftIntake.setPosition(0);
            else if (gamepad2.left_trigger != 0) leftIntake.setPosition(1);

            if (gamepad2.right_bumper) rightIntake.setPosition(0);
            else if (gamepad2.right_trigger != 0) rightIntake.setPosition(1);


            if (gamepad2.dpad_up) linearStop.setPosition(1);
            else if (gamepad2.dpad_down) linearStop.setPosition(0);

            if (gamepad2.dpad_right) {
                liftStop.setPosition(0.66);
                linearLift.setPower(-1);
            }
            else if (gamepad2.dpad_left) liftStop.setPosition(0);

/*
            if (gamepad1.a) RGB_status = RGB_status + 0.01;
            else if (gamepad1.b) RGB_status = RGB_status - 0.01;
            RGB.setPosition(RGB_status);
*/

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
