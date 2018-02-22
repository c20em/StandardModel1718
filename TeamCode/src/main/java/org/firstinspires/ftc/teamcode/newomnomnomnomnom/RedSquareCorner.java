package org.firstinspires.ftc.teamcode.newomnomnomnomnom;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by student on 2/16/18.
 */

@Autonomous(name="Red Square Corner", group="Autonomous")
public class RedSquareCorner extends BaseAutoFunctions {

    @Override
    public void runOpMode() throws InterruptedException {
        declare();
        waitForStart();

        liftIn.setPosition(.9);             //Relic Blocker
        elbowServo.setPosition(.2);         //Relic arm up
        sleep(1000);                         //breif pause so that wall servo does not interfere with relic arm release
        wallServo.setPosition(.3);          //Wall servo out

        while(opModeIsActive()) {
            telemetry.addLine("made it this far");
            telemetry.update();

            jewel(false);
            sleep(600);


            driveforTime(-.5, 1200);
            sleep(200);
            turn(0.8, 480);
            sleep(300);
            driveforTime(-.4, 500);
            sleep(300);
            driveforTime(.4, 300);
            sleep(200);
            flipOut();
            sleep(1000);
            driveforTime(-.3, 400);
            sleep(200);
            driveforTime(.4, 200);
            sleep(200);
            flipIn();
            sleep(1000);
            driveforTime(.5, 500);
            NomNomNom.setPower(-1);
            sleep(200);
            nomDrive(.7);
            sleep(400);
            nom();
            StopDriving();
            sleep(200);
            nomDrive(-.7);
            sleep(800);
            StopDriving();





            break;





        }
    }
}
