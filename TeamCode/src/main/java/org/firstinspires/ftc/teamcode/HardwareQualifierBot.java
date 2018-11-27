package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Hardware;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

public class HardwareQualifierBot {
    DcMotor leftDrive1, leftDrive2, rightDrive1, rightDrive2,
            arm, hang, extend;

    CRServo collection;
    Servo pivot;


    HardwareQualifierBot() {

    }


    void init(HardwareMap hwMap) {

        leftDrive1 = hwMap.get(DcMotor.class, "ld1");
        leftDrive2 = hwMap.get(DcMotor.class, "ld2");
        rightDrive1 = hwMap.get(DcMotor.class, "rd1");
        rightDrive2 = hwMap.get(DcMotor.class, "rd2");

        leftDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDrive2.setDirection(DcMotorSimple.Direction.REVERSE);

        arm = hwMap.get(DcMotor.class, "arm");
        hang = hwMap.get(DcMotor.class, "hang");
        extend = hwMap.get(DcMotor.class, "ext");

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        collection = hwMap.get(CRServo.class, "clt");
        pivot = hwMap.get(Servo.class, "pvt");

        collection.setPower(-0.05);
        pivot.setPosition(0.1);

    }
}
