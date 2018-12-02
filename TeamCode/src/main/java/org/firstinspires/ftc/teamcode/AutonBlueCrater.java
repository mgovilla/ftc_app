package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;

/**
 * Autonomous for the Blue Side starting in front of the Crater, parking in the near crater
 */
@Autonomous(name="Test")

public class AutonBlueCrater extends org.firstinspires.ftc.teamcode.Autonomous {


    public void runOpMode() {
        robot = new HardwareQualifierBot(hardwareMap, telemetry);
        robot.init();

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        }
        encoderReset();
        runWithoutEncoders();

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
                telemetry.addData("GO LEFT", goldPos);
                telemetry.update();
                turnIMU(20);
                driveInches(35);
                turnIMU(-40);

                driveInches(20);

                break;
            case 1:
                //Go straight
                telemetry.addData("GO CENTER", goldPos);
                telemetry.update();
                break;
            default:
                //Turn to the Right
                telemetry.addData("GO RIGHT", goldPos);
                telemetry.update();
                break;
        }
        sleep(2500);

        unlatch();

        // Lower down onto the field
        // Turn to unlatch
        // Picture of minerals
        // Choose Path
        // case: 0 Knock down First Mineral
        // case: 1 Knock down Second Mineral
        // case: 2 Knock down third mineral

        // Maneuver around other minerals
        // Drive to Depot
        // Deposit Marker
        // Drive back to Crater and Park

    }
}
