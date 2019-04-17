package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_GOLD_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_SILVER_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.TFOD_MODEL_ASSET;

abstract public class Auton extends LinearOpMode {

    HardwareQualifierBot robot;

    private static double GEAR_RATIO = 1.5;
    private static double CIRCUMFERENCE = 4.0 * Math.PI;
    private static double TICKS_PER_REV = 1120;

    private static double CAMERA_THRESHOLD = 220.0; // Line where the minerals are considered sampling or in the crater for autonomous

    private int goldMineralPosition = 3; //default to the right

    private static final String VUFORIA_KEY = "AT1IefX/////AAAAGe7qBWfq2E2BmnY2RCELAlIGMKLOUEEXYPeKxkKghQTsCPGGs0xtZTnLSPrE6ExcuJ6WcUeXawPMLqGyDWP9a1YhinvHIaS+8cXJrelXtK+CShbo+KOY8QyygETMDOLqlYFUWEDjsK0Gc4TQFh1XaaawNIq+1kepx5DHU4+ODL0+qLfuZGrd5rDygrfNqx1U3q72j+2l2M4iPH6cgcZk2ydxCKjPLybctUK73xCFdeEfTtwWKklES/BOGss+bRFpSaBfY71SldEZi08fd/FRIQh9bXI5DxaV8mIgEehsQR4TUMkjJfpHRdQwJeEti7hq4isMki3wAUsHTEZ2PJuvrvB5toFEOoJR7Qf9sxERlRIA";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    void setPower(double power) {
        robot.rightDrive1.setPower(power);
        robot.rightDrive2.setPower(power);
        robot.leftDrive1.setPower(power);
        robot.leftDrive2.setPower(power);
    }

    void setPower(double right, double left) {
        robot.rightDrive1.setPower(right);
        robot.rightDrive2.setPower(right);
        robot.leftDrive1.setPower(left);
        robot.leftDrive2.setPower(left);
    }

    void encoderReset() {
        robot.rightDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(10);
    }

    void runWithoutEncoders() {
        robot.rightDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(10);
    }

    /**
     *
     * @param distance
     */
    void driveInches(double distance) {
        double power;

        double counts = ((-distance / (GEAR_RATIO * CIRCUMFERENCE))) * TICKS_PER_REV;
        double target = robot.rightDrive1.getCurrentPosition() + counts;

        if(robot.rightDrive1.getCurrentPosition() > target) { //go forward
            while (Math.abs(robot.rightDrive1.getCurrentPosition() - target) > 45 && !isStopRequested()) {
                telemetry.addData("Motor Right 1", robot.rightDrive1.getCurrentPosition());
                telemetry.addData("Target", target);
                telemetry.update();

                power = Range.clip((robot.rightDrive1.getCurrentPosition() - target) / 350.0, 0.25, 0.6);
                setPower(power);

                if(robot.hang.getCurrentPosition() > -250.0) {
                    robot.hang.setPower(0.0);
                }
            }
        } else {
            while (Math.abs(robot.rightDrive1.getCurrentPosition() - target) > 45 && !isStopRequested()) {
                telemetry.addData("Motor Right 1", robot.rightDrive1.getCurrentPosition());
                telemetry.addData("Target", target);
                telemetry.update();

                power = Range.clip((robot.rightDrive1.getCurrentPosition() - target) / 350.0, -0.6, -0.25);
                setPower(power);

                if(robot.hang.getCurrentPosition() > -250.0) {
                    robot.hang.setPower(0.0);
                }
            }
        }


        setPower(0.0);

    }

    void driveInches(double distance, boolean tilt) {
        double power;

        double counts = ((-distance / (GEAR_RATIO * CIRCUMFERENCE))) * TICKS_PER_REV;
        double target = robot.rightDrive1.getCurrentPosition() + counts;

        if(robot.rightDrive1.getCurrentPosition() > target) { //go forward
            while (Math.abs(robot.rightDrive1.getCurrentPosition() - target) > 45 && !isStopRequested()) {
                telemetry.addData("Motor Right 1", robot.rightDrive1.getCurrentPosition());
                telemetry.addData("Target", target);
                telemetry.update();


                power = Range.clip((robot.rightDrive1.getCurrentPosition() - target) / 350.0, 0.25, 0.6);
                if(!tilt) {
                    setPower(power);
                } else {
                    setPower(power, power + 0.15);
                }
            }
        } else {
            while (Math.abs(robot.rightDrive1.getCurrentPosition() - target) > 45 && !isStopRequested()) {
                telemetry.addData("Motor Right 1", robot.rightDrive1.getCurrentPosition());
                telemetry.addData("Target", target);
                telemetry.update();

                power = Range.clip((robot.rightDrive1.getCurrentPosition() - target) / 350.0, -0.6, -0.25);
                if(!tilt) {
                    setPower(power);
                } else {
                    setPower(power, power + 0.15);
                }
            }
        }


        setPower(0.0);

    }

    void armToPos(DcMotor motor, double angle) {

        double power;

        angle = angle/360.0;
        double counts = ((angle) * (10752.0));
        double target = motor.getCurrentPosition() + counts;

        while (Math.abs(motor.getCurrentPosition() - target) > 100 && !isStopRequested()) {
            telemetry.addData("Motor Pos", motor.getCurrentPosition());
            telemetry.addData("Target", target);
            telemetry.update();
            power = Range.clip((target - motor.getCurrentPosition()) / 60.0, .4, .8);
            motor.setPower(-power);
        }

        motor.setPower(0.0);
    }

    void armToPos(DcMotor motor, double angle, boolean reverse) {

        double power;

        angle = angle/360.0;
        double counts = ((angle) * (10752.0));
        double target = motor.getCurrentPosition() + counts;

        while (Math.abs(motor.getCurrentPosition() - target) > 100 && !isStopRequested()) {
            telemetry.addData("Motor Pos", motor.getCurrentPosition());
            telemetry.addData("Target", target);
            telemetry.update();
            power = Range.clip((motor.getCurrentPosition() - target) / 60.0, .4, .8);
            motor.setPower(power);
        }

        motor.setPower(0.0);
    }

    void hangToPos(DcMotor motor, double counts) {

        double power;
        double target = motor.getCurrentPosition() + counts;

        while (Math.abs(motor.getCurrentPosition() - target) > 45 && !isStopRequested()) {
            telemetry.addData("Motor Pos", motor.getCurrentPosition());
            telemetry.addData("Target", target);
            telemetry.update();
            power = Range.clip( (-Math.abs(motor.getCurrentPosition() - target)) / 100.0 , -1.0, -0.7);
            motor.setPower(power);
        }

        motor.setPower(0.0);

    }


    void unlatch() {

        armToPos(robot.arm, 20);
        robot.hardStop.setPosition(1.0);
        robot.pivot.setPosition(0.8);

        hangToPos(robot.hang, -16800); //34000 for 40 motor

        robot.hardStop.setPosition(0.6);
        robot.arm.setPower(.5);
        sleep(375);
        robot.arm.setPower(0.0);

        setPower(.25);
        sleep(100);
        setPower(0.0);

        sleep(250);
    }

    int getGoldPosition() {
        double startTime = getRuntime();

        if (tfod != null) {
            tfod.activate();
        }

        while (goldMineralPosition == 3 && getRuntime() - startTime < 5 && opModeIsActive()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 2) {

                        int[] minerals = new int[2];

                        for (int i = 0; i < 2; i++) {
                            minerals[i] = (int) updatedRecognitions.get(i).getLeft();
                        }

                        if (minerals[1] > minerals[0]) {
                            /*
                             *       Mineral 1 is to the right of mineral 0
                             */

                            if (updatedRecognitions.get(1).getLabel().equals(LABEL_GOLD_MINERAL)) {
                                telemetry.addData("Gold Mineral Position", "Right");
                                goldMineralPosition = 2;
                            } else if (updatedRecognitions.get(0).getLabel().equals(LABEL_GOLD_MINERAL)) {
                                telemetry.addData("Gold Mineral Position", "Center");
                                goldMineralPosition = 1;
                            } else {
                                telemetry.addData("Gold Mineral Position", "Left");
                                goldMineralPosition =0;
                            }
                        } else {
                            /*
                             *       Mineral 0 is to the right of mineral 1
                             */

                            if (updatedRecognitions.get(0).getLabel().equals(LABEL_GOLD_MINERAL)) {
                                telemetry.addData("Gold Mineral Position", "Right");
                                goldMineralPosition = 2;
                            } else if (updatedRecognitions.get(1).getLabel().equals(LABEL_GOLD_MINERAL)) {
                                telemetry.addData("Gold Mineral Position", "Center");
                                goldMineralPosition = 1;
                            } else {
                                telemetry.addData("Gold Mineral Position", "Left");
                                goldMineralPosition = 0;
                            }
                        }

                    }
                    telemetry.update();
                }
            }
        }

        if (tfod != null)
            tfod.shutdown();

        return goldMineralPosition;
    }

    int getGoldPosition(boolean Crater) {
        List<Recognition> threshRecognitions = new ArrayList<>(); // List that holds the objects below the threshold
        double startTime = getRuntime();
        boolean recognized = false;

        if (tfod != null) {
            tfod.activate();
        }

        while (goldMineralPosition == 3 && getRuntime() - startTime < 5 && opModeIsActive()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                if(!recognized) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", threshRecognitions.size());

                        for (Recognition r : updatedRecognitions) {
                            telemetry.addData("Top Boundary", (int) r.getTop());
                            if (r.getTop() > CAMERA_THRESHOLD) {
                                threshRecognitions.add(r);
                            }
                        }
                        telemetry.update();
                        if (threshRecognitions.size() == 2) {

                            recognized = true;
                            int[] minerals = new int[2];

                            for (int i = 0; i < 2; i++) {
                                minerals[i] = (int) threshRecognitions.get(i).getLeft();
                            }

                            if (minerals[1] > minerals[0]) {
                                /*
                                 *       Mineral 1 is to the right of mineral 0
                                 */

                                if (updatedRecognitions.get(1).getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    goldMineralPosition = 2;
                                } else if (updatedRecognitions.get(0).getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    goldMineralPosition = 1;
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    goldMineralPosition =0;
                                }
                            } else {
                                /*
                                 *       Mineral 0 is to the right of mineral 1
                                 */

                                if (updatedRecognitions.get(0).getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    goldMineralPosition = 2;
                                } else if (updatedRecognitions.get(1).getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    goldMineralPosition = 1;
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    goldMineralPosition = 0;
                                }
                            }

                            telemetry.update();
                        }
                    }
                    threshRecognitions.clear();

                }
            }
        }

        if (tfod != null)
            tfod.shutdown();

        return goldMineralPosition;
    }

    void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    void turnIMU(float target) {
        robot.updatePosition();

        double startTime = getRuntime();
        float initial = robot.pos.firstAngle;
        float current = initial;
        target = target + initial;

        double deltaAngle = 0, initTime, deltaTime;
        double i = 0;

        while (Math.abs((current) - (target)) > 1.0 && getRuntime() - startTime < 4 && opModeIsActive()) {
            initTime = getRuntime();

            robot.updatePosition();
            current = robot.pos.firstAngle; //getHeading


            double power = Range.clip((Math.abs((current - target) / (100.0)) + i), .275, .7);


            if (current < target) { //Must turn left
                setPower(power, -power);
            } else if (current > target) { //Must turn right
                setPower(-power, power);
            }

            deltaTime = getRuntime() - initTime;

            if (Math.abs(current - target) < 30)
                i += .0095 * Math.abs(current - target) * deltaTime;

            if (i > 0.3) {
                i = 0.3;
            }


            if(robot.hang.getCurrentPosition() > -250) {
                robot.hang.setPower(0.0);
            }

            telemetry.addData("toGo", String.format("%.01f deg", current - target));
            telemetry.addData("power", power);
            telemetry.addData("i", i);

            telemetry.update();
        }
        setPower(0.0);
    }

    void turnIMU(float target, double constant) {
        robot.updatePosition();

        double startTime = getRuntime();
        float initial = robot.pos.firstAngle;
        float current = initial;
        target = target + initial;

        double deltaAngle = 0, initTime, deltaTime;
        double i = 0;

        while (Math.abs((current) - (target)) > 1.0 && getRuntime() - startTime < 4 && opModeIsActive()) {
            initTime = getRuntime();

            robot.updatePosition();
            current = robot.pos.firstAngle; //getHeading


            double power = Range.clip((Math.abs((current - target) / (constant)) + i), .275, .5);


            if (current < target) { //Must turn left
                setPower(power, -power);
            } else if (current > target) { //Must turn right
                setPower(-power, power);
            }

            deltaTime = getRuntime() - initTime;

            if (Math.abs(current - target) < 30)
                i += .0095 * Math.abs(current - target) * deltaTime;

            if (i > 0.3) {
                i = 0.3;
            }

            if(robot.hang.getCurrentPosition() > -250) {
                robot.hang.setPower(0.0);
            }

            telemetry.addData("toGo", String.format("%.01f deg", current - target));
            telemetry.addData("power", power);
            telemetry.addData("i", i);

            telemetry.update();
        }
        setPower(0.0);
    }

    void wallFollow(double distanceThreshold, int distance) {
        double error, angleError;
        double counts = ((-distance / (1.5 * 12.56))) * 1120;
        double target = robot.rightDrive1.getCurrentPosition() + counts;
        double rightPower, leftPower;
        robot.updatePosition();

        double initAngle = robot.pos.firstAngle;

        while(Math.abs(robot.rightDrive1.getCurrentPosition() - target) > 50 && opModeIsActive()) {
            robot.updatePosition();
            error = robot.dist.getDistance(DistanceUnit.INCH) - distanceThreshold;
            angleError = initAngle - robot.pos.firstAngle;

            if(angleError > 25) {

                turnIMU((float) angleError);

            } else {
                if (Math.abs(error) > 0.2) {
                    rightPower = 0.3 - Range.clip(error / 4.0, -0.2, 0.2);
                    leftPower = 0.3 + Range.clip(error / 4.0, -0.2, 0.2);
                } else {
                    rightPower = 0.4;
                    leftPower = 0.4;
                }

                setPower(rightPower, leftPower);
            }

            telemetry.addData("range", String.format("%.01f in", robot.dist.getDistance(DistanceUnit.INCH)));
            telemetry.addData("angle Error", String.format("%.01f deg", angleError));
            telemetry.update();

        }

        setPower(0.0);

    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
