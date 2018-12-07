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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Mainbot Code", group="Linear Opmode")
public class Mainbot_Code extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontleftDrive = null;
    private DcMotor frontrightDrive = null;
    private DcMotor backleftDrive = null;
    private DcMotor backrightDrive = null;

    private DcMotor liftDrive = null;

    private TouchSensor topLift = null;
    private TouchSensor lowerLift = null;

    private DcMotor intakeMotor = null;

    private DcMotor linearMotor = null;

    private TouchSensor linearSlideOut = null;
    private TouchSensor linearSlideIn = null;

    double linearPower = 0;

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

        /*intakeMotor = hardwareMap.get(DcMotor.class, "im");

        linearMotor = hardwareMap.get(DcMotor.class, "lm");

        linearSlideOut = hardwareMap.get(TouchSensor.class, "lso");
        linearSlideIn = hardwareMap.get(TouchSensor.class, "lsi");*/

        double frontleftDrivePower = 0;
        double frontrightDrivePower = 0;

        double backleftDrivePower = 0;
        double backrightDrivePower = 0;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            frontleftDrivePower = -gamepad1.left_stick_y /*For driving forward/backward*/ + gamepad1.right_stick_x /*Turning*/ + gamepad1.left_stick_x /*Strafing*/;
            frontrightDrivePower = -gamepad1.left_stick_y /*For driving forward/backward*/ - gamepad1.right_stick_x /*Turning*/ - gamepad1.left_stick_x /*Strafing*/;

            backleftDrivePower = -gamepad1.left_stick_y /*For driving forward/backward*/ + gamepad1.right_stick_x /*Turning*/ - gamepad1.left_stick_x /*Strafing*/;
            backrightDrivePower = -gamepad1.left_stick_y /*For driving forward/backward*/ - gamepad1.right_stick_x /*Turning*/ + gamepad1.left_stick_x /*Strafing*/;


            frontleftDrivePower = Range.clip(frontleftDrivePower, -1, 1);
            frontrightDrivePower = Range.clip(frontrightDrivePower, -1, 1);

            backleftDrivePower = Range.clip(backleftDrivePower, -1, 1);
            backrightDrivePower = Range.clip(backrightDrivePower, -1, 1);


            frontleftDrive.setPower(frontleftDrivePower);
            frontrightDrive.setPower(frontrightDrivePower);

            backleftDrive.setPower(backleftDrivePower);
            backrightDrive.setPower(backrightDrivePower);

            liftDrive.setPower(-gamepad2.right_stick_y);

            /*intakeMotor.setPower(Range.clip(gamepad1.right_trigger - gamepad1.left_trigger, -1, 1));

            if (linearSlideIn.isPressed() && linearPower < 0);
            else linearMotor.setPower(linearPower);

            if (linearSlideOut.isPressed() && linearPower > 0);
            else linearMotor.setPower(linearPower);*/

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
