package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

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
import org.firstinspires.ftc.robotcore.external.navigation.Position;
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

    private HardwareMap hardwareMap;
    private Telemetry telemetry;


    DcMotor leftDrive1, leftDrive2, rightDrive1, rightDrive2,
            arm, hang, extend;

    CRServo collection;
    Servo pivot;

    BNO055IMU imu;

    Orientation pos;

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
        pivot.setPosition(0.2);

        //********IMU********\\

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //updatePosition();
    }

    void updatePosition() {

        if(imu != null) {
            pos = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }

    }
}
