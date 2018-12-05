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
        robot.imu.initialize(robot.parameters);

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
            default:
                //Turn to the left
                telemetry.addData("GO LEFT", goldPos);
                telemetry.update();
                turnIMU(15);
                driveInches(35);
                sleep(100);
                driveInches(-5);
                turnIMU(-40);

                driveInches(20);

                robot.marker.setPosition(1.0);
                driveInches(-5);

                turnIMU((-robot.pos.firstAngle-45));
                break;

            case 1:
                //Go straight

                telemetry.addData("GO CENTER", goldPos);
                telemetry.update();
                turnIMU(-5);
                driveInches(50);

                robot.marker.setPosition(1.0);
                driveInches(-10);

                turnIMU((-robot.pos.firstAngle-45));
                break;

            case 2:
                //Turn to the Right
                telemetry.addData("GO RIGHT", goldPos);
                telemetry.update();
                turnIMU(-30);
                sleep(100);

                driveInches(40);
                sleep(100);
                driveInches(-10);
                turnIMU(40);

                driveInches(30);

                robot.marker.setPosition(1.0);
                driveInches(-5);

                turnIMU((-robot.pos.firstAngle-45));
                break;
        }
        sleep(2500);

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
