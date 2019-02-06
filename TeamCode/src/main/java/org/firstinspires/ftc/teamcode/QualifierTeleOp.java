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
 * Main TeleOp for 12/8 Qualifier
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

    private boolean isPressed   = false,
                    isPressed2  = false,
                    storing     = false,
                    collecting  = false,             // false = delivery, true = collection
                    lifting,
                    descending  = false,
                    spitting    = false,
                    hanging     = false,
                    CMode       = false;

    private int     trigger     = 1,
                    trigger2    = 0;

    private double  armAngle    = 0.0,
                    extendAngle = 0.0,
                    armTarget   = 160.0,
                    extendTgt   = 850.0;

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

        double hangPower    = gamepad2.right_trigger;
        double armPower     = gamepad1.right_trigger;
        double extendPower  = gamepad1.left_trigger;

        double drive        = -gamepad1.left_stick_y + -gamepad2.left_stick_y;
        double turn         =  gamepad1.right_stick_x + gamepad2.right_stick_x;

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
        }

        /*
         *
         *  Setting Power to Hang
         *
         */
        if(hangPower > 0.2) {                                                                       // Pulling Down
            robot.hang.setPower(hangPower);
            hanging = false;
        } else if(gamepad2.right_bumper) {                                                          // Go up
            hanging = true;
        } else if(hanging && robot.hang.getCurrentPosition() > -16800) {
            robot.hang.setPower(-1.0);
        } else {
            robot.hang.setPower(0.0);
        }


        /*
         *  Toggling the Collection Mode
         *  Collect rotates between .5 and .7 MAX  40 --> .5 80 --> .65
         *  slope = .00375 (3/800)
         *  intercept = .35
         *  Delivery rotates
         *
         */

        //collection mode is even (%2 == 0)

        if (gamepad1.right_trigger > .1 && gamepad1.left_bumper) {                                  // This is code for the Arm Movement
            lifting = true;                                                                         // When the button is pressed, start lifting and
            trigger2 = 3;                                                                           // Set the trigger to flag CMode false
        }

        CMode = (trigger2 % 2 == 0);

        // Code to prevent adding to trigger multiple times while holding the button
        if(isPressed2 && !gamepad1.x) {
            trigger2++;
        }

        isPressed2 = gamepad1.x;
        //60 motor: 1680 counts per revolution

        armAngle = (robot.arm.getCurrentPosition()/(-5040.0)) * 360.0;
        //armAngle = (-115 * robot.potentiometer.getVoltage()) + 195;

        // Main if statement for determining Pivot Position
        if(CMode) {                                                                                 // Collection Mode
            if(gamepad1.b) {                                                                        // To push the other minerals away
                robot.pivot.setPosition(.8);
                spitting = true;                                                                    // To spit particles out with collection
                collecting = false;                                                                 // Prevent multiple instances of setPosition
            } else {
                spitting = false;
                collecting = true;                                                                  // After the button is released, resume collection mode
            }

            if(armAngle > 25 && armAngle < 80 && collecting) {                                      // Between the collecting arm range
                robot.pivot.setPosition
                        (Range.clip((3.0*armAngle / 800.0) + 0.34, .15, .65));    //Scale Servo value based on angle
            }
        } else {                                                                                    // Storage Mode
            if(gamepad1.dpad_down && armAngle > 150) {                                              // To deliver the minerals after the arm achieves a certain angle
                storing = false;
                robot.pivot.setPosition(.7);
            } else {
                storing = true;
            }

            if(armAngle > 20 && storing) {                                                          // To bring the pivot up
                robot.pivot.setPosition(Range.clip(armAngle / 180, .15, .875));
            }
        }


        /*
         *
         *  Rotating the Arm
         *
         */

        if(lifting && (Math.abs(extendAngle - extendTgt) > 40.0 || armAngle < armTarget)) {         // Only lift while angle is below a certain value
            if(gamepad1.right_bumper || (armAngle > 150 && armPower > 0)) {
                lifting = false;                                                                    // To cancel lifting if necessary
            }

            if(Math.abs(extendAngle - extendTgt) > 200.0) {
                robot.extend.setPower
                        (Range.clip(-(Math.abs(extendAngle - extendTgt)) / 500.0, -1.0, -0.1));
            } else {
                robot.extend.setPower(0.0);
            }

            if(armAngle < armTarget) {
                armPower = Range.clip((armTarget - armAngle) / 40.0, .375, 0.75);
                robot.arm.setPower(armPower);                                                       // Setting power for lifting
            } else {
                robot.arm.setPower(0.0);
            }

        } else {
            // lifting = false;                                                                     // Stop lifting
            if(gamepad1.right_trigger > .1) {                                                       // To finish lifting for the scoring
                robot.arm.setPower(gamepad1.right_trigger);
                descending = false;
            } else if(gamepad1.right_bumper) {                                                      // Put the arm back down
                robot.arm.setPower(-0.3);
                descending = true;
            } else {
                robot.arm.setPower(0);
                descending =false;
            }

            /*
             *
             *  Extending the Arm
             *
             */
            if(extendPower > 0.2) {
                robot.extend.setPower(extendPower);
            } else if(gamepad1.left_bumper) {
                robot.extend.setPower(-1.0);
            } else if (descending) {
                robot.extend.setPower(0.2);                                                         // Run the Extension at the same rate as arm
            } else {
                robot.extend.setPower(0.0);
            }

        }

        /*
         *
         *  Toggling the Collection
         *
         */
        if(gamepad1.y || spitting) {                                                                // In case we need to spit out or push minerals
            robot.collection.setPower(-1.0);
        } else {
            if(trigger % 2 == 0) {
                robot.collection.setPower(1.0);                                                     // Turn collection on
            } else {
                robot.collection.setPower(-0.05);                                                   // Stop Collection
            }
        }


        // Code to prevent adding to trigger multiple times while holding the button
        if(isPressed && !gamepad1.a) {
            trigger++;
        }

        isPressed = gamepad1.a;


        extendAngle = robot.extend.getCurrentPosition();

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Arm Angle", armAngle);
        telemetry.addData("Servo Position", Range.clip(armAngle / 180, .15, .875));
        telemetry.addData("Arm Position", robot.arm.getCurrentPosition());
        telemetry.addData("Hang Position", robot.hang.getCurrentPosition());
        telemetry.addData("Rd1 Position", robot.rightDrive1.getCurrentPosition());
        telemetry.addData("Ld1 Position", robot.leftDrive1.getCurrentPosition());
        telemetry.addData("Extend Enc", extendAngle);
        telemetry.addData("Potentiometer Input", robot.potentiometer.getVoltage());


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
