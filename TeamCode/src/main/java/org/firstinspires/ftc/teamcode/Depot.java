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
            default: //DONE
                //Turn to the left
                telemetry.addData("GO LEFT", goldPos);
                telemetry.update();
                driveInches(9);
                turnIMU(13);
                driveInches(40);
                sleep(100);
                driveInches(-4);
                turnIMU(-78);

                driveInches(22);

                robot.marker.setPosition(1.0);
                sleep(250);
                driveInches(-5);

                //turnIMU((-robot.pos.firstAngle-45));
                driveInches(-60, true);

                robot.hang.setPower(1.0);
                while(robot.hang.getCurrentPosition() < -250.0 && opModeIsActive()) {
                    telemetry.addData("hang", robot.hang.getCurrentPosition());
                }
                robot.hang.setPower(0.0);


                break;

            case 1: //DONE
                //Go straight

                telemetry.addData("GO CENTER", goldPos);
                telemetry.update();
                //turnIMU(-5);
                driveInches(4);
                sleep(200);

                turnIMU(-robot.pos.firstAngle);

                driveInches(55);

                robot.marker.setPosition(1.0);
                driveInches(-2);

                robot.updatePosition();
                turnIMU((-robot.pos.firstAngle-45));
                driveInches(-57,  true);

                robot.hang.setPower(1.0);
                while(robot.hang.getCurrentPosition() < -250.0 && opModeIsActive()) {
                    telemetry.addData("hang", robot.hang.getCurrentPosition());
                }
                robot.hang.setPower(0.0);

                break;

            case 2: //DONE
                //Turn to the Right
                telemetry.addData("GO RIGHT", goldPos);
                telemetry.update();

                driveInches(6);
                robot.updatePosition();
                turnIMU(-65);
                sleep(100);

                driveInches(45);
                sleep(100);
                driveInches(-8);

                turnIMU(90);

                driveInches(40);

                robot.marker.setPosition(1.0);
                //driveInches(-5);

                turnIMU(-87);
                //driveInches(-24);

                driveInches(-55, true);
                robot.hang.setPower(1.0);
                while(robot.hang.getCurrentPosition() < -250.0 && opModeIsActive()) {
                    telemetry.addData("hang", robot.hang.getCurrentPosition());
                }
                robot.hang.setPower(0.0);

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
