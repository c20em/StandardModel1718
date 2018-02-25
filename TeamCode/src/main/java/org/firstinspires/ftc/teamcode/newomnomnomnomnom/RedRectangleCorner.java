package org.firstinspires.ftc.teamcode.newomnomnomnomnom;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by student on 2/16/18.
 */

@Autonomous(name = "Red Rectangle Corner", group = "Autonomous")
public class RedRectangleCorner extends BaseAutoFunctions {

    @Override
    public void runOpMode() throws InterruptedException {

        declare();
        initVuforia();
        waitForStart();

        liftIn.setPosition(.9);             //Relic Blocker
        elbowServo.setPosition(ELBOW_UP);         //Relic arm up
        sleep(1000);                         //breif pause so that wall servo does not interfere with relic arm release
        wallServo.setPosition(.3);          //Wall servo out

        while (opModeIsActive()) {
            telemetry.addLine("made it this far");
            telemetry.update();
            RelicRecoveryVuMark hi = pictograph();
            telemetry.addData("vuMark:", hi);
            telemetry.update();
            jewel(false);
            sleep(600);
            driveforTime(-.6, 600);
            sleep(500);
            strafeforTime(.8, 700);
            sleep(200);
            driveforTime(-.5, 1000);
            sleep(200);
            driveforTime(.3, 600);
            sleep(200);
            turn(.3, 200);
            sleep(200);
            flipOut();
            driveforTime(-.3, 500);
            sleep(200);
            driveforTime(.3, 400);
            sleep(200);
            strafeforTime(.8, 200);
            sleep(100);
            flipIn();
            sleep(200);
            driveforTime(-.3, 700);
            driveforTime(.5, 200);

            break;
        }
    }
}
