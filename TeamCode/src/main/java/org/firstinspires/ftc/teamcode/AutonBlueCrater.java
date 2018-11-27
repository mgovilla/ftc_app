package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Autonomous for the Blue Side starting in front of the Crater, parking in the near crater
 */
public class AutonBlueCrater extends LinearOpMode {
    HardwareQualifierBot robot = new HardwareQualifierBot(hardwareMap, telemetry);

    public void runOpMode() {
         robot.init();

         waitForStart();

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
