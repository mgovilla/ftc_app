package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;

/**
 * Autonomous starting in front of the Crater, parking in the near crater
 */
@Autonomous(name="Crater")
//@Disabled
public class Crater extends Auton {


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


        int goldPos = getGoldPosition(true);
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

                driveInches(9);
                turnIMU(13);

                // Go forward until the mineral is pushed off
                driveInches(20);
                sleep(200);

                // Drive back for space to turn without hitting silver mineral
                driveInches(-17);
                sleep(200);

                // Turn left until the front of the robot is toward the wall (~90 deg)
                turnIMU(85 - robot.pos.firstAngle);
                sleep(100);

                // Drive forward to the wall (~50-60 inches)
                driveInches(42);
                sleep(100);

                // Turn parallel to the wall (~45 degrees more)
                turnIMU(135 - robot.pos.firstAngle);

                break;

            case 1:
                //Go straight

                telemetry.addData("GO CENTER", goldPos);
                telemetry.update();

                driveInches(4);
                sleep(200);

                turnIMU(-robot.pos.firstAngle);

                // Go forward until the mineral is pushed off
                driveInches(25);
                sleep(200);

                // Drive back for space to turn without hitting silver mineral
                driveInches(-15);
                sleep(200);

                // Turn left until the front of the robot is toward the wall (~90 deg)
                turnIMU(90 - robot.pos.firstAngle);
                sleep(100);

                // Drive forward to the wall (~50-60 inches)
                driveInches(42);
                sleep(100);

                // Turn parallel to the wall (~45 degrees more)
                turnIMU(135 - robot.pos.firstAngle);

                // Drive along the wall (avoid the silver mineral) (~60 inches)


                // Score the marker


                // Drive back and park


                break;

            case 2:
                //Turn to the Right
                telemetry.addData("GO RIGHT", goldPos);
                telemetry.update();

                driveInches(6);
                robot.updatePosition();
                turnIMU(-62);
                sleep(100);

                // Go forward until the mineral is pushed off
                driveInches(25);
                sleep(200);

                // Drive back for space to turn without hitting silver mineral
                driveInches(-15);
                sleep(200);

                // Turn left until the front of the robot is toward the wall (~90 deg)
                turnIMU(92 - robot.pos.firstAngle);
                sleep(100);

                // Drive forward to the wall (~50-60 inches)
                driveInches(42);
                sleep(100);

                // Turn parallel to the wall (~45 degrees more)
                turnIMU(135 - robot.pos.firstAngle);

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
