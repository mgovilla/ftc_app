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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Main TeleOp for 4/12 Scrimmage
 * Tank Drive
 * Encoder Arm Angle Tracking
 *
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="New Arm test", group="Iterative Opmode")

public class TeleOp extends OpMode {
    // Declare OpMode members.
    private HardwareQualifierBot robot;

    private boolean isPressed   = false,
                    isPressed2  = false,
                    isPressed3  = false,
                    storing     = false,
                    scoring     = false,
                    collecting  = false,             // false = delivery, true = collection
                    lifting     = false,
                    descending  = false,
                    spitting    = false,
                    hanging     = false,
                    CMode       = false;

    private int     trigger     = 1,
                    trigger2    = 0,
                    trigger3    = 1;

    private double  armAngle    = 0.0,
                    extendAngle = 0.0,
                    armTarget   = -50.0,
                    extendTgt   = 440.0,
                    CltRev      = -1.0,
                    extendPower = 0.0,
                    armPower = 0.0,
                    offset = 0.0;

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

        armAngle = (robot.arm.getCurrentPosition()/(10752.0)) * 360.0 + offset;
        extendAngle = robot.extend.getCurrentPosition();

        /*
         * DRIVE Block
         */

        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        double drive        = -gamepad1.left_stick_y + -gamepad2.left_stick_y;
        double turn         = gamepad1.right_stick_x + gamepad2.right_stick_x;

        leftPower           = Range.clip(drive + turn, -1.0, 1.0);
        rightPower          = Range.clip(drive - turn, -1.0, 1.0);

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
        } // Drive Control

        /*
         * DRIVE Block
         */

        /*
         * COLLECTION Block
         */

        if(gamepad1.y || spitting) {                                                                // In case we need to spit out or push minerals
            robot.collection.setPower(1.0);
        } else {
            if(trigger % 2 == 0) {
                robot.collection.setPower(-1.0);                                                    // Turn collection on
            } else {
                robot.collection.setPower(0.0);                                                     // Stop Collection
            }
        }
        // Code to prevent adding to trigger multiple times while holding the button
        if(isPressed && !gamepad1.a) {
            trigger++;
        }

        isPressed = gamepad1.a; // Collection Control

        /*
         * COLLECTION Block
         */

        /*
         * SERVO CONTROL Block
         */

        CMode = (trigger2 % 2 == 0);

        // Code to prevent adding to trigger multiple times while holding the button
        if(isPressed2 && (!gamepad1.x && !gamepad2.x)) {
            trigger2++;
        }

        isPressed2 = (gamepad1.x || gamepad2.x);
        //60 motor: 1680 counts per revolution

        // Main if statement for determining Pivot Position
        if(CMode) {                                                                                 // Collection Mode
            if(gamepad1.b) {                                                                        // To push the other minerals away
                robot.pivot.setPosition(0.675);
                spitting = true;                                                                    // To spit particles out with collection
                collecting = false;                                                                 // Prevent multiple instances of setPosition
            } else {
                robot.pivot.setPosition(0.4);
                spitting = false;
                collecting = true;                                                                  // After the button is released, resume collection mode
            }
        } else {                                                                                    // Storage Mode
            if((gamepad1.dpad_down || gamepad2.dpad_down) && armAngle < -48.0) {                      // To deliver the minerals after the arm achieves a certain angle
                storing = false;
                robot.pivot.setPosition(0.5);
                isPressed3 = true;
            } else {
                storing = true;
            }

            if(armAngle < 35 && storing) {                                                          // To bring the pivot up
                robot.pivot.setPosition
                        (Range.clip((-4.0*armAngle / 900.0) + 0.6, .15, 0.9));
            }
        } // Collection and storage mode control

        /*
         * SERVO CONTROL Block
         */




        if(gamepad1.right_bumper) {
            //robot.extend.setPower(1.0);
            extendPower = 1.0;
        } else if(gamepad1.left_bumper) {
            //robot.extend.setPower(-0.5);
            extendPower = -0.75;
        } else {
            extendPower = 0.0;
        }

        if (robot.extend.getCurrentPosition() > -250 && extendPower < 0.0) {
            //robot.extend.setPower(0.0);
            extendPower = 0.0;
        }

        robot.extend.setPower(extendPower);


        if (gamepad1.right_trigger > .1 && gamepad1.left_bumper) {                                  // This is code for the Arm Movement
            lifting = true;                                                                         // When the button is pressed, start lifting and
            trigger2 = 3;                                                                                        // Set the trigger to flag CMode false
        }

        if(lifting && (Math.abs(extendAngle - extendTgt) > 40.0 || armAngle > armTarget)) {         // Only lift while angle is above a certain value **
            if((gamepad1.left_trigger > 0 || gamepad2.left_trigger > 0)||(armAngle < -50.0 && armPower > 0)) {
                lifting = false;                                                                    // To cancel lifting if necessary
            }

            /*if(Math.abs(extendAngle - extendTgt) > 100.0) {
                if(extendAngle > extendTgt) {
                    robot.extend.setPower
                            (Range.clip((Math.abs(extendAngle - extendTgt)) / 260.0, 0.3, 1.0));
                } else {
                    robot.extend.setPower
                            (Range.clip((-Math.abs(extendAngle - extendTgt)) / 260.0, -1.0, -0.3));
                }
            } else if (extendPower < -0.2) {
                robot.extend.setPower(extendPower);
            } else {
                robot.extend.setPower(0.0);
            }*/

            if(armAngle > armTarget) {
                armPower = Range.clip((Math.abs(armTarget - armAngle)) / 60.0, 0.375, 1.0);
                robot.arm.setPower(armPower);                                                       // Setting power for lifting
            } else {
                robot.arm.setPower(0.0);
                armPower = 0.0;
            }

        } else {
            // lifting = false;                                                                     // Stop lifting
            if(gamepad1.right_trigger > 0.2) {
                robot.arm.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.2 || gamepad2.left_trigger > 0.2) {
                robot.arm.setPower(-Range.clip(gamepad1.left_trigger + gamepad2.left_trigger, 0, 1.0));
            } else {
                robot.arm.setPower(0.0);
            }

            if(armAngle < -48.0) {
                scoring = true;
            }
        }


        /*
         *  Setting Power to Hang
         */

        if(gamepad2.right_bumper) {                                                                 // Pulling Down
            robot.hang.setPower(1.0);
            hanging = false;
        } else if(gamepad2.right_trigger > 0.2) {                                                   // Go up
            hanging = true;
        } else if(hanging && robot.hang.getCurrentPosition() > -16800) {
            robot.hang.setPower(-1.0);
        } else {
            robot.hang.setPower(0.0);
        } // HANG control

        /*
         *  Setting Power to Hang
         */



        if(gamepad2.b) {
            robot.hardStop.setPosition(1.0);
        } else {
            robot.hardStop.setPosition(0.6);
        }


        if(isPressed3 && !gamepad1.right_stick_button) {
            offset = offset + (-50.0 - armAngle);
        }
        isPressed3 = gamepad1.right_stick_button;

        telemetry.addData("Arm Angle", armAngle);
        telemetry.addData("Potentiometer Voltage", robot.potentiometer.getVoltage());
        telemetry.addData("Extend Position", robot.extend.getCurrentPosition());

        telemetry.addData("Servo position", robot.pivot.getPosition());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
