package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

    private TouchSensor linearSlideOut = null;
    private TouchSensor linearSlideIn = null;

    private DcMotor linearMotorLift = null;

    private DcMotor slideMotor = null;
    
    private Servo leftIntake = null;
    private Servo rightIntake = null;

    /*private Servo linearStop = null;
    private Servo liftStop = null;*/

     private double linearPower = 0;

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

        linearSlideOut = hardwareMap.get(TouchSensor.class, "lso");
        linearSlideIn = hardwareMap.get(TouchSensor.class, "lsi");

        linearMotorLift = hardwareMap.get(DcMotor.class, "ll");

        slideMotor = hardwareMap.get(DcMotor.class, "sm");
        
        leftIntake = hardwareMap.get(Servo.class, "li");
        rightIntake = hardwareMap.get(Servo.class, "ri");

/*
        linearStop = hardwareMap.get(Servo.class, "linearStop");

        liftStop = hardwareMap.get(Servo.class, "liftStop");
*/

        //Gyro is "imu"

        frontrightDrive.setDirection(DcMotor.Direction.REVERSE);
        backrightDrive.setDirection(DcMotor.Direction.REVERSE);


        frontleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        linearMotorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        linearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double frontleftDrivePower;
        double frontrightDrivePower;

        double backleftDrivePower;
        double backrightDrivePower;
/*

        linearStop.setPosition(0);
        liftStop.setPosition(0);
*/

//        double RGB_status = 0;

        waitForStart();
        runtime.reset();

        double linearLiftMax = linearMotorLift.getCurrentPosition() + 147;

        double linearLiftPower;

        boolean turbo;

        linearMotorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {
            turbo = gamepad1.right_bumper;

            frontleftDrivePower = -gamepad1.left_stick_y /*For driving forward/backward*/ + gamepad1.right_stick_x /*Turning*/ - gamepad1.left_stick_x /*Strafing*/;
            frontrightDrivePower = -gamepad1.left_stick_y /*For driving forward/backward*/ - gamepad1.right_stick_x /*Turning*/ + gamepad1.left_stick_x /*Strafing*/;

            backleftDrivePower = -gamepad1.left_stick_y /*For driving forward/backward*/ + gamepad1.right_stick_x /*Turning*/ + gamepad1.left_stick_x /*Strafing*/;
            backrightDrivePower = -gamepad1.left_stick_y /*For driving forward/backward*/ - gamepad1.right_stick_x /*Turning*/ - gamepad1.left_stick_x /*Strafing*/;


            frontleftDrivePower = Range.clip(frontleftDrivePower * Math.abs(frontleftDrivePower), -1, 1);
            frontrightDrivePower = Range.clip(frontrightDrivePower * Math.abs(frontrightDrivePower), -1, 1);

            backleftDrivePower = Range.clip(backleftDrivePower * Math.abs(backleftDrivePower), -1, 1);
            backrightDrivePower = Range.clip(backrightDrivePower * Math.abs(backrightDrivePower), -1, 1);

            if (!turbo) {
                frontleftDrivePower = frontleftDrivePower / 2;
                frontrightDrivePower = frontrightDrivePower / 2;

                backleftDrivePower = backleftDrivePower / 2;
                backrightDrivePower = backrightDrivePower / 2;
            }

            frontleftDrive.setPower(frontleftDrivePower);
            frontrightDrive.setPower(frontrightDrivePower);

            backleftDrive.setPower(backleftDrivePower);
            backrightDrive.setPower(backrightDrivePower);

            if (!turbo) {
                if (gamepad1.dpad_up) linearMotorLift.setPower(0.5);
                else if (gamepad1.dpad_down) linearMotorLift.setPower(-0.5);
                else linearMotorLift.setPower(0);
            } else {
                if (gamepad1.dpad_up) linearMotorLift.setPower(1);
                else if (gamepad1.dpad_down) linearMotorLift.setPower(-1);
                else linearMotorLift.setPower(0);
            }

            if (gamepad1.dpad_right) linearMotor.setPower(1);
            else if (gamepad1.dpad_left) linearMotor.setPower(-1);
            else linearMotor.setPower(0);

            if (gamepad1.left_bumper) leftIntake.setPosition(0);
            else if (gamepad1.left_trigger != 0) leftIntake.setPosition(1);

            if (gamepad1.right_bumper) rightIntake.setPosition(1);
            else if (gamepad1.right_trigger != 0) rightIntake.setPosition(0);

            if (gamepad1.y && !linearSlideOut.isPressed()) slideMotor.setPower(1);
            else if (gamepad1.a && !linearSlideIn.isPressed()) slideMotor.setPower(-1);
            else slideMotor.setPower(0);

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
