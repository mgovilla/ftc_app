package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Locale;

@TeleOp(name = "Mecanum")
@Disabled
public class MecanumTest extends LinearOpMode {

    DcMotor leftDrive1, leftDrive2, rightDrive1, rightDrive2, extendC, extendL, collect;

    // Extending positive is retraction



    Servo clt, dlv, door;

    boolean isPressed, isPressed2;
    int trigger, trigger2;

    @Override
    public void runOpMode() throws InterruptedException {

        //telemetry.addData("Status", "Initialized");

        leftDrive1 = hardwareMap.get(DcMotor.class, "ld1");
        leftDrive2 = hardwareMap.get(DcMotor.class, "ld2");
        rightDrive1 = hardwareMap.get(DcMotor.class, "rd1");
        rightDrive2 = hardwareMap.get(DcMotor.class, "rd2");

        extendC = hardwareMap.get(DcMotor.class, "extC");
        extendL = hardwareMap.get(DcMotor.class, "extL");

        collect = hardwareMap.get(DcMotor.class, "tubing");


        leftDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDrive2.setDirection(DcMotorSimple.Direction.REVERSE);

        extendC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clt = hardwareMap.get(Servo.class, "clt");
        dlv = hardwareMap.get(Servo.class, "dlv");
        door = hardwareMap.get(Servo.class, "door");


        waitForStart();


        while (opModeIsActive()) {
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4.0;
            double rightX = gamepad1.right_stick_x;

            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            leftDrive1.setPower(v1);
            rightDrive1.setPower(v2);
            leftDrive2.setPower(v3);
            rightDrive2.setPower(v4);


            if(gamepad1.right_trigger > 0.2) {
                extendC.setPower(gamepad1.right_trigger);
            } else if(gamepad1.right_bumper) {
                extendC.setPower(-0.5); //extendC out
            } else {
                extendC.setPower(0.0);
            }

            /*
             *
             *  Toggling the Collection
             *
             */
            if(gamepad1.y) {                                                                        // In case we need to spit out or push minerals
                collect.setPower(1.0);
            } else {
                if(trigger % 2 == 0) {
                    collect.setPower(-1.0);                                                         // Turn collection on
                } else {
                    collect.setPower(0.0);                                                          // Stop Collection
                }
            }


            // Code to prevent adding to trigger multiple times while holding the button
            if(isPressed && !gamepad1.a) {
                trigger++;
            }

            isPressed = gamepad1.a;


            if(trigger2 % 2 == 0) {
                clt.setPosition(0.75);
                door.setPosition(0.5);
            } else {
                if(gamepad1.left_bumper) {
                    door.setPosition(0.0);
                    clt.setPosition(0.38);
                } else {
                    clt.setPosition(0.5);
                }
            }


            // Code to prevent adding to trigger multiple times while holding the button
            if(isPressed2 && !gamepad1.x) {
                trigger2++;
            }

            isPressed2 = gamepad1.x;

            /*
             *
             *  Extend Delivery
             *
             */
            if(gamepad2.left_trigger > 0.2) {
                extendL.setPower(gamepad2.right_trigger);
            } else if(gamepad2.right_trigger > 0.2) {
                extendL.setPower(-gamepad2.left_trigger);
            } else {
                extendL.setPower(0.0);
            }

            if(gamepad2.a) {
                dlv.setPosition(0.31);
            } else if(gamepad2.b) {
                dlv.setPosition(0.41);
            } else if(gamepad2.y) {
                dlv.setPosition(0.80);
            }



            telemetry.addData("rd1 %.01f", rightDrive1.getCurrentPosition());
            telemetry.addData("ld1 %.01f", leftDrive1.getCurrentPosition());

            telemetry.addData("rd2 %.01f", rightDrive2.getCurrentPosition());
            telemetry.addData("ld2 %.01f", leftDrive2.getCurrentPosition());
            //telemetry.addData("lift", delivery.getCurrentPosition());
            //telemetry.addData("collection", collection.getCurrentPosition());


            telemetry.update();
        }
    }
}


