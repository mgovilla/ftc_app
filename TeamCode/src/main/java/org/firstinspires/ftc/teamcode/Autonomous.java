package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_GOLD_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_SILVER_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.TFOD_MODEL_ASSET;

abstract public class Autonomous extends LinearOpMode {
    HardwareQualifierBot robot;

    private static double GEAR_RATIO = 1.5;
    private static double CIRCUMFERENCE = 4.0 * Math.PI;
    private static double TICKS_PER_REV = 1120;

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

                power = Range.clip((robot.rightDrive1.getCurrentPosition() - target) / 350.0, 0.15, 0.6);
                setPower(power);
            }
        } else {
            while (Math.abs(robot.rightDrive1.getCurrentPosition() - target) > 45 && !isStopRequested()) {
                telemetry.addData("Motor Right 1", robot.rightDrive1.getCurrentPosition());
                telemetry.addData("Target", target);
                telemetry.update();

                power = Range.clip((target - robot.rightDrive1.getCurrentPosition()) / 350.0, -0.6, -0.15);
                setPower(power);
            }
        }


        while (Math.abs(robot.rightDrive1.getCurrentPosition() - target) > 45 && !isStopRequested()) {
            telemetry.addData("Motor Right 1", robot.rightDrive1.getCurrentPosition());
            telemetry.addData("Target", target);
            telemetry.update();

            power = Range.clip(robot.rightDrive1.getCurrentPosition() - target / 350, -0.6, 0.6);

            setPower(power);
        }

        setPower(0.0);

    }

    void armToPos(DcMotor motor, double angle) {

        double power;
        angle = angle/360.0;
        double counts = ((angle) * (-5040.0));
        double target = motor.getCurrentPosition() + counts;

        while (Math.abs(motor.getCurrentPosition() - target) > 45 && !isStopRequested()) {
            telemetry.addData("Motor Pos", motor.getCurrentPosition());
            telemetry.addData("Target", target);
            telemetry.update();
            power = Range.clip(target - motor.getCurrentPosition() / 350, .5, .8);
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
            power = Range.clip(target - motor.getCurrentPosition() / 100 , -1.0, -0.7);
            motor.setPower(power);
        }

        motor.setPower(0.0);

    }


    void unlatch() {
        armToPos(robot.arm, 20);

        hangToPos(robot.hang, -16900);

        setPower(.25);
        sleep(100);
        setPower(0.0);

        turnIMU(35);
        sleep(200);

        driveInches(4);
        sleep(200);

        turnIMU(-robot.pos.firstAngle);

        sleep(2500);

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
                                telemetry.addData("Gold Mineral Position", "Center");
                                goldMineralPosition = 1;
                            } else if (updatedRecognitions.get(0).getLabel().equals(LABEL_GOLD_MINERAL)) {
                                telemetry.addData("Gold Mineral Position", "Left");
                                goldMineralPosition = 0;
                            } else {
                                telemetry.addData("Gold Mineral Position", "Right");
                                goldMineralPosition = 2;
                            }
                        } else {
                            /*
                             *       Mineral 0 is to the right of mineral 1
                             */

                            if (updatedRecognitions.get(0).getLabel().equals(LABEL_GOLD_MINERAL)) {
                                telemetry.addData("Gold Mineral Position", "Center");
                                goldMineralPosition = 1;
                            } else if (updatedRecognitions.get(1).getLabel().equals(LABEL_GOLD_MINERAL)) {
                                telemetry.addData("Gold Mineral Position", "Left");
                                goldMineralPosition = 0;
                            } else {
                                telemetry.addData("Gold Mineral Position", "Right");
                                goldMineralPosition = 2;
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

        double lastLeftPos = robot.leftDrive1.getCurrentPosition(), lastRightPos = robot.rightDrive1.getCurrentPosition();
        double deltaLeft = 0, deltaRight = 0;
        double deltaAngle = 0, initTime, deltaTime;
        double i = 0;

        while (Math.abs((current) - (target)) > .75 && getRuntime() - startTime < 4 && opModeIsActive()) {
            initTime = getRuntime();

            robot.updatePosition();
            current = robot.pos.firstAngle; //getHeading
            telemetry.addData("toGo", current - target);
            telemetry.update();

            double power = Range.clip((Math.abs((current - target) / (target * 1.1)) + i), .25, .5);


            if (current < target) { //Must turn left
                setPower(power + ((deltaLeft - deltaRight)), -power);
            } else if (current > target) { //Must turn right
                setPower(-power, power + (deltaRight - deltaLeft));
            }

            deltaLeft = Math.abs(robot.leftDrive1.getCurrentPosition() - lastLeftPos) / 1000;
            deltaRight = Math.abs(robot.rightDrive1.getCurrentPosition() - lastRightPos) / 1000;
            deltaTime = -initTime;

            lastLeftPos = robot.leftDrive1.getCurrentPosition();
            lastRightPos = robot.rightDrive1.getCurrentPosition();

            if (Math.abs(current - target) < 30)
                i += .005 * (current - target) * deltaTime;

            if (i > .3) {
                i = .3;
            }
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
