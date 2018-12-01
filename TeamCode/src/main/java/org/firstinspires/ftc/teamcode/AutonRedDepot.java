package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;

/**
 * Autonomous for the Blue Side starting in front of the Depot, parking in the far crater
 */
@Autonomous(name="RedDepot")
public class AutonRedDepot extends org.firstinspires.ftc.teamcode.Autonomous {

    public void runOpMode() {
        robot = new HardwareQualifierBot(hardwareMap, telemetry);

        robot.init();
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        }


        waitForStart();

        int goldPos = getGoldPosition();
        // Lower down onto the field
        unlatch();

        // Choose Path
        // case: 0 Knock down Left Mineral
        // case: 1 Knock down Center Mineral
        // case: 2 Knock down Right mineral

        switch (goldPos) {
            case 0:
                //Turn to the left
                turnIMU(35);
                driveInches(10);
                turnIMU(-robot.pos.firstAngle);

                driveInches(10);


                telemetry.addData("GO LEFT", goldPos);
                telemetry.update();
                break;
            case 1:
                //Go straight
                driveInches(17);
                telemetry.addData("GO CENTER", goldPos);
                telemetry.update();
                break;
            default:
                //Turn to the Right

                turnIMU(-35);
                driveInches(10);
                turnIMU(robot.pos.firstAngle);

                driveInches(10);
                telemetry.addData("GO RIGHT", goldPos);
                telemetry.update();
                break;
        }

        //Deposit Marker

        //Park in RED crater

    }
}
