/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Main TeleOp for 12/3 Qualifier
 * Tank Drive
 * Encoder Arm Angle Tracking
 * Potentiometer: 1.65 v at 0deg and 0 v at 185deg
 * linear: 115 degrees per volt
 *
 */

@TeleOp(name="TeleOp", group="Iterative Opmode")

public class QualifierTeleOp extends OpMode {
    // Declare OpMode members.
    private HardwareQualifierBot robot;

    private boolean isPressed = false,
                    isPressed2 = false,
                    delivering = false; //false = delivery, true = collection

    private int trigger = 1,
                trigger2 = 0;

    private double armAngle = 0;

    private ElapsedTime runtime = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot = new HardwareQualifierBot(hardwareMap, telemetry);

        robot.init();
        telemetry.addData("Status", "Initialized");
        // Tell the driver that initialization is complete.
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        double hangPower = gamepad2.right_trigger;
        double armPower = gamepad1.right_trigger;
        double extendPower = gamepad1.left_trigger;

        double drive = -gamepad2.left_stick_y;
        double turn  =  gamepad2.right_stick_x;

        leftPower    = Range.clip(drive + turn, -1.0, 1.0) + -gamepad1.left_stick_y;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) + -gamepad1.right_stick_y;


        // Send calculated power to wheels
        if(Math.abs(leftPower) > 0.2) {
            robot.leftDrive1.setPower(leftPower);
            robot.leftDrive2.setPower(leftPower);
        } else {
            robot.leftDrive1.setPower(0.0);
            robot.leftDrive2.setPower(0.0);
        }

        if(Math.abs(rightPower) > 0.2) {
            robot.rightDrive1.setPower(rightPower);
            robot.rightDrive2.setPower(rightPower);
        } else {
            robot.rightDrive1.setPower(0.0);
            robot.rightDrive2.setPower(0.0);
        }

        /*
         *
         *  Setting Power to Hang
         *
         */
        if(hangPower > 0.2) {
            robot.hang.setPower(-hangPower);
        } else if(gamepad2.right_bumper) {
            robot.hang.setPower(1.0);
        } else {
            robot.hang.setPower(0.0);
        }

        /*
         *
         *  Rotating the Arm
         *
         */
        if(armPower > 0.2) {
            robot.arm.setPower(armPower);
        } else if(gamepad1.right_bumper) {
            robot.arm.setPower(-0.3);
        } else {
            robot.arm.setPower(0.0);
        }

        /*
         *
         *  Extending the Arm
         *
         */
        if(extendPower > 0.2) {
            robot.extend.setPower(extendPower);
        } else if(gamepad1.left_bumper) {
            robot.extend.setPower(-0.3);
        } else {
            robot.extend.setPower(0.0);
        }

        /*
         *
         *  Toggling the Collection Itself
         *
         */
        if(trigger % 2 == 0) {
            robot.collection.setPower(1.0); //Collection is ON
        } else if (gamepad1.y) {
            robot.collection.setPower(-1.0);
        } else {
            robot.collection.setPower(-0.05);
        }

        if(isPressed && !gamepad1.a) {
            trigger++;
        }

        isPressed = gamepad1.a;

        /*
         *  Toggling the Collection Mode
         *  Collect rotates between .5 and .7 MAX  40 --> .5 80 --> .65
         *  slope = .00375 (3/800)
         *  intercept = .35
         *  Delivery rotates
         *
         */

        //collection mode is even (%2 == 0)
        boolean CMode = (trigger2 % 2 == 0);


        if(isPressed2 && !gamepad1.x) {
            trigger2++;
        }

        isPressed2 = gamepad1.x;
        //60 motor: 1680 counts per revolution

        armAngle = (robot.arm.getCurrentPosition()/(-5040.0)) * 360.0;
        //armAngle = (-115 * robot.potentiometer.getVoltage()) + 202;

        if(CMode) { // Collection Mode
            if (armAngle > 30 && armAngle < 80) {
                robot.pivot.setPosition(Range.clip((3.0*armAngle / 800.0) + 0.35, .15, .65));
            }
        } else {    // Storage Mode

            if(gamepad1.dpad_down && armAngle > 150) {
                delivering = true;
                robot.pivot.setPosition(.7);
            } else {
                delivering = false;
            }

            if(armAngle > 20 && !delivering) {
                robot.pivot.setPosition(Range.clip(armAngle / 180, .15, .875));
            }

        }



        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Arm Angle", armAngle);
        telemetry.addData("Servo Position", Range.clip(armAngle / 180, .15, .875));
        telemetry.addData("Arm Position", robot.arm.getCurrentPosition());
        telemetry.addData("Hang Position", robot.hang.getCurrentPosition());
        telemetry.addData("Rd1 Position", robot.rightDrive1.getCurrentPosition());
        telemetry.addData("Ld1 Position", robot.leftDrive1.getCurrentPosition());

        telemetry.addData("Potentiometer Input", robot.potentiometer.getVoltage());

        telemetry.addData("Trigger", trigger2);


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
