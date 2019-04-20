package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Autonomous starting in front of the Crater, parking in the near crater
 */
@Autonomous(name="Crater")
//@Disabled
public class Crater extends Auton {


    public void runOpMode() {
        robot = new HardwareQualifierBot(hardwareMap, telemetry);
        robot.init();
        robot.pivot.setPosition(1.0);
        robot.imu.initialize(robot.parameters);
        sleep(100);
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        }
        encoderReset();
        runWithoutEncoders();

        waitForStart();


        int goldPos = getGoldPosition(true);
        double thresh;
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

                turnIMU(36, 50);

                robot.hang.setPower(1.0);
                // Go forward until the mineral is pushed off
                driveInches(30);
                sleep(200);

                // Drive back for space to turn without hitting silver mineral
                driveInches(-10);
                sleep(200);

                // Turn left until the front of the robot is toward the wall (~90 deg)
                robot.updatePosition();
                turnIMU(-97 - robot.pos.firstAngle);
                sleep(100);

                // Drive forward to the wall (~50-60 inches)
                driveInches(-30);
                sleep(100);

                // Turn parallel to the wall (~45 degrees more)
                robot.updatePosition();
                turnIMU(-50 - robot.pos.firstAngle);


                driveInches(-10.0);
                scoreMarkerAndPark(25);
                break;

            case 1:
                //Go straight

                telemetry.addData("GO CENTER", goldPos);
                telemetry.update();

                turnIMU(20, 50);
                sleep(200);

                driveInches(4);
                sleep(200);

                robot.updatePosition();
                turnIMU(-robot.pos.firstAngle, 50);

                robot.hang.setPower(1.0);
                // Go forward until the mineral is pushed off
                driveInches(25);
                sleep(200);

                // Drive back for space to turn without hitting silver mineral
                driveInches(-15);
                sleep(200);
                // Turn left until the front of the robot is toward the wall (~90 deg)
                robot.updatePosition();
                turnIMU(-97 - robot.pos.firstAngle);
                sleep(100);

                // Drive forward to the wall (~50-60 inches)
                driveInches(-44);
                sleep(100);

                // Turn parallel to the wall (~45 degrees more)
                robot.updatePosition();
                turnIMU(-45 - robot.pos.firstAngle);

                driveInches(-6);
                scoreMarkerAndPark(25);


                break;

            case 2:
                //Turn to the Right
                telemetry.addData("GO RIGHT", goldPos);
                telemetry.update();

                turnIMU(20, 25.0);
                driveInches(6);

                robot.hang.setPower(1.0);
                robot.updatePosition();
                turnIMU(-67);
                sleep(100);

                // Go forward until the mineral is pushed off
                driveInches(28);
                sleep(200);

                // Drive back for space to turn without hitting silver mineral
                driveInches(-13);
                sleep(200);

                // Turn left until the front of the robot is toward the wall (~90 deg)
                robot.updatePosition();
                turnIMU(-95 - robot.pos.firstAngle);
                sleep(100);

                // Drive forward to the wall (~50-60 inches)
                driveInches(-43);
                sleep(100);

                // Turn parallel to the wall (~45 degrees more)

                robot.updatePosition();
                turnIMU(-48 - robot.pos.firstAngle);

                driveInches(-8.0);
                scoreMarkerAndPark(23);

                break;
        }

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
