package org.firstinspires.ftc.teamcode.ArchAuto3;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by student on 2/18/18.
 */

@Autonomous(name = "Blue Rectangle Corner", group = "Autonomous")
@Disabled
public class BlueRectangleCorner extends BaseAutoFunctions {
    @Override
    public void runOpMode() throws InterruptedException {
        declare();
        waitForStart();

        liftIn.setPosition(.9);                                          //Relic Blocker
        elbowServo.setPosition(ELBOW_UP);         //Relic arm up
        sleep(1000);                         //breif pause so that wall servo does not interfere with relic arm release
        wallServo.setPosition(.3);          //Wall servo out

        while(opModeIsActive()) {
            telemetry.addLine("made it this far");
            telemetry.update();

            jewel(true);
            sleep(600);
            driveforTime(.6, 600);
            sleep(200);
            turn(.8, 900);
            sleep(200);
            //strafeLeft(.9, 400, false);
            sleep(200);
            driveforTime(-.5, 1000);
            sleep(200);
            driveforTime(.3, 400);
            sleep(200);
            flipOut();
            driveforTime(-.3, 400);
            sleep(200);
        }
    }
}
