package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Autonomous for the Blue Side starting in front of the Depot, parking in the far crater
 */
@Autonomous(name="BlueDepot")
public class AutonBlueDepot extends LinearOpMode {

    HardwareQualifierBot robot = new HardwareQualifierBot(hardwareMap, telemetry);


    public void runOpMode() {
        robot.init();

        waitForStart();

        int goldPos = robot.getGoldPosition();
        // Lower down onto the field
        // robot.unlatch();
        // Turn to unlatch

        // Choose Path
        // case: 0 Knock down Left Mineral
        // case: 1 Knock down Center Mineral
        // case: 2 Knock down Right mineral

        switch (goldPos) {
            case 0:
                //Turn to the left
                telemetry.addData("GO LEFT", goldPos);
                telemetry.update();
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

        //Deposit Marker

        //Park in RED crater

    }
}
