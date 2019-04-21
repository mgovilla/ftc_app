package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;

/**
 * Autonomous starting in front of the Depot, parking in the far crater
 */
@Autonomous(name="Depot")
public class Depot extends Auton {

    public void runOpMode() {
        robot = new HardwareQualifierBot(hardwareMap, telemetry);
        robot.init();

        robot.pivot.setPosition(1.0);

        robot.imu.initialize(robot.parameters);

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        }

        encoderReset();
        runWithoutEncoders();

        waitForStart();

        int goldPos = 0;

        try {
            goldPos = getGoldPosition();
        } catch(Exception e) {

        }

        // Lower down onto the field
        unlatch();

        // Choose Path
        // case: 0 Knock down Left Mineral
        // case: 1 Knock down Center Mineral
        // case: 2 Knock down Right mineral

        switch (goldPos) {
            default: //DONE
                //Turn to the left
                telemetry.addData("GO LEFT", goldPos);
                telemetry.update();

                turnIMU(36, 50);

                robot.hang.setPower(1.0);
                // Go forward until the mineral is pushed off
                driveInches(30);
                sleep(200);

                // Drive back for space to turn without hitting silver mineral
                driveInches(-13);
                sleep(200);

                robot.updatePosition();
                turnIMU(175 - robot.pos.firstAngle, true, 0.02);

                while(robot.hang.getCurrentPosition() < -250.0 && opModeIsActive()) {
                    telemetry.addData("hang", robot.hang.getCurrentPosition());
                }
                robot.hang.setPower(0.0);
                scoreMarker();

                robot.updatePosition();
                turnIMU(87 - robot.pos.firstAngle);

                driveInches(46);

                turnIMU(130 - robot.pos.firstAngle, true, 0.02);

                driveInches(5);
                ArmPark();
                break;

            case 1: //DONE
                //Go straight

                telemetry.addData("GO CENTER", goldPos);
                telemetry.update();
                //turnIMU(-5);
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
                driveInches(-8);
                sleep(200);

                robot.updatePosition();
                turnIMU(175 - robot.pos.firstAngle, true, 0.02);

                while(robot.hang.getCurrentPosition() < -250.0 && opModeIsActive()) {
                    telemetry.addData("hang", robot.hang.getCurrentPosition());
                }
                robot.hang.setPower(0.0);
                scoreMarker();

                robot.updatePosition();
                turnIMU(87 - robot.pos.firstAngle);

                driveInches(50);

                turnIMU(130 - robot.pos.firstAngle, true, 0.02);

                driveInches(5);
                ArmPark();

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
                driveInches(-15);
                sleep(200);

                robot.updatePosition();
                turnIMU(-180 - robot.pos.firstAngle, true, 0.03);

                while(robot.hang.getCurrentPosition() < -250.0 && opModeIsActive()) {
                    telemetry.addData("hang", robot.hang.getCurrentPosition());
                }
                robot.hang.setPower(0.0);

                scoreMarker();

                robot.updatePosition();
                turnIMU(84 - robot.pos.firstAngle);

                driveInches(52);

                turnIMU(130 - robot.pos.firstAngle, true, 0.02);

                driveInches(5);
                ArmPark();

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
