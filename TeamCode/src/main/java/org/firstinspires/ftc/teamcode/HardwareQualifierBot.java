package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static java.lang.Runtime.getRuntime;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_GOLD_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_SILVER_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.TFOD_MODEL_ASSET;

public class HardwareQualifierBot {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    DcMotor leftDrive1, leftDrive2, rightDrive1, rightDrive2,
            arm, hang, extend;

    CRServo collection;
    Servo pivot;

    BNO055IMU imu;

    int goldMineralPosition = 3; //default to the right

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

    private final double NANOSECONDS_PER_SECOND = TimeUnit.SECONDS.toNanos(1);

    HardwareQualifierBot(HardwareMap hM, Telemetry tM) {
        hardwareMap = hM;
        telemetry = tM;
    }


    void init() {

        leftDrive1 = hardwareMap.get(DcMotor.class, "ld1");
        leftDrive2 = hardwareMap.get(DcMotor.class, "ld2");
        rightDrive1 = hardwareMap.get(DcMotor.class, "rd1");
        rightDrive2 = hardwareMap.get(DcMotor.class, "rd2");

        leftDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDrive2.setDirection(DcMotorSimple.Direction.REVERSE);

        arm = hardwareMap.get(DcMotor.class, "arm");
        hang = hardwareMap.get(DcMotor.class, "hang");
        extend = hardwareMap.get(DcMotor.class, "ext");

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        collection = hardwareMap.get(CRServo.class, "clt");
        pivot = hardwareMap.get(Servo.class, "pvt");

        collection.setPower(-0.05);
        pivot.setPosition(0.15);

        //********IMU********\\
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile  = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled       = true;
        parameters.loggingTag           = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod(hardwareMap);
        }

    }

    public void setPower(double power) {
        rightDrive1.setPower(power);
        rightDrive2.setPower(power);
        leftDrive1.setPower(power);
        leftDrive2.setPower(power);
    }

    public void setPowerSides(double right, double left) {
        rightDrive1.setPower(right);
        rightDrive2.setPower(right);
        leftDrive1.setPower(left);
        leftDrive2.setPower(left);
    }


    void unlatch() {

    }

    int getGoldPosition() {
        double startTime = System.nanoTime();

        if (tfod != null) {
            tfod.activate();
        }

        while(goldMineralPosition == 3 && ((System.nanoTime() - startTime)/ NANOSECONDS_PER_SECOND) < 5) {
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

    private void initVuforia() {
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
    private void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public void turnIMU(float target) {

        double startTime = System.nanoTime();
        Orientation Pos = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float initial = Pos.firstAngle;
        float current = initial;
        target = target + initial;

        double lastLeftPos = leftDrive1.getCurrentPosition(), lastRightPos = rightDrive1.getCurrentPosition();
        double deltaLeft = 0, deltaRight = 0;
        double deltaAngle = 0, initTime, deltaTime;
        double i = 0;

        while(Math.abs((current) - (target)) > .75 && (System.nanoTime()-startTime)/NANOSECONDS_PER_SECOND < 4) {
            initTime = System.nanoTime()/NANOSECONDS_PER_SECOND;

            Pos = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            current = Pos.firstAngle; //getHeading
            telemetry.addData("toGo", current - target);
            telemetry.update();

            double power = Range.clip((Math.abs((current - target) / (target*1.1))+i), .35, .5);


            if(current < target) { //Must turn left
                setPowerSides(power + ((deltaLeft-deltaRight)), -power);
            } else if(current > target) { //Must turn right
                setPowerSides(-power, power + (deltaRight-deltaLeft));
            }

            deltaLeft = Math.abs(leftDrive1.getCurrentPosition() - lastLeftPos)/1000;
            deltaRight = Math.abs(rightDrive1.getCurrentPosition() - lastRightPos)/1000;
            deltaTime =  - initTime;

            lastLeftPos = leftDrive1.getCurrentPosition();
            lastRightPos = rightDrive1.getCurrentPosition();

            if(Math.abs(current - target) < 30)
                i += .005*(current - target)*deltaTime;

            if(i>.3) {
                i=.3;
            }
        }

    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}
