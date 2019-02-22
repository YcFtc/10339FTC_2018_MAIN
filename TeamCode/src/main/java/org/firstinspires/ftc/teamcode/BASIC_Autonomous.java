package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="BASIC AUTONOMOUS", group="z")
public class BASIC_Autonomous extends LinearOpMode {

    private DcMotor frontleftDrive = null;
    private DcMotor frontrightDrive = null;
    private DcMotor backleftDrive = null;
    private DcMotor backrightDrive = null;

    private DcMotor slideMotor = null;
    private TouchSensor linearSlideIn = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        frontleftDrive = hardwareMap.get(DcMotor.class, "fl");
        frontrightDrive = hardwareMap.get(DcMotor.class, "fr");

        backleftDrive = hardwareMap.get(DcMotor.class, "bl");
        backrightDrive = hardwareMap.get(DcMotor.class, "br");

        slideMotor = hardwareMap.get(DcMotor.class, "sm");

        linearSlideIn = hardwareMap.get(TouchSensor.class, "lsi");

        frontleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if (opModeIsActive()) {

            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //1120 * 3 wheel turns
            slideMotor.setTargetPosition(1120 * 3);

            slideMotor.setPower(1);

            sleep(5000);


/*
            frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontleftDrive.setTargetPosition();
            frontrightDrive.setTargetPosition();

            backleftDrive.setTargetPosition();
            backrightDrive.setTargetPosition();
*/


//            RUN_DRIVE(-0.3, 4);

            frontleftDrive.setPower(-0.5);
            frontrightDrive.setPower(-0.5);

            backleftDrive.setPower(-0.5);
            backrightDrive.setPower(-0.5);

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
        }
    }
    private void RUN_DRIVE(double ForwardPower, int ForwardWheelTurns){

        if (ForwardWheelTurns != 0){
//            int ticks_to_turn = (int) Math.rint(ForwardWheelTurns * 537);/*Rounding WheelTurns * 537 to the closest integer value and then casting/converting it to an int*/


            int ticks_to_turn = 537 / ForwardWheelTurns;

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

}
