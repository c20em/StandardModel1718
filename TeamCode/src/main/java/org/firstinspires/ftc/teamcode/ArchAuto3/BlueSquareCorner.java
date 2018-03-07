package org.firstinspires.ftc.teamcode.ArchAuto3;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by student on 2/16/18.
 */

@Autonomous(name="Blue Square Corner", group="Autonomous")
@Disabled
public class BlueSquareCorner extends BaseAutoFunctions {

    @Override
    public void runOpMode() throws InterruptedException {
        declare();
        initVuforia();
        waitForStart();

        liftIn.setPosition(.9);             //Relic Blocker
        elbowServo.setPosition(ELBOW_UP);         //Relic arm up
        sleep(1000);                         //breif pause so that wall servo does not interfere with relic arm release
        wallServo.setPosition(.78);          //Wall servo out

        while(opModeIsActive()) {
            telemetry.addLine("made it this far");
            telemetry.update();

            jewel(true);
            sleep(600);

            ////////////////////t0fp
            //GO FOR FIRST GLYPH:::::::::::::::;;;f>x/,////////b//b////sss     sss ssb                              aabab   a                                                                                                ,b               k.
            ////////////////////

            //Drive to Cryptobox
            driveforTime(0.5, 1200); // would change this for vuforia`
            sleep(200);
            turn(0.8, 450);
            sleep(300);

            //SERVO flip out SEQUENCE
            driveforTime(-.7, 800);        //align with box .....?
            sleep(300);
            driveforTime(.4, 500);
            sleep(200);
            turn(.4, 200);
            sleep(200);
            flipOut();
            sleep(1000);
            driveforTime(.3, 200);
            driveforTime(-.3, 1000);
            sleep(200);
            driveforTime(.4, 400);
            sleep(200);
            strafeforTime(.5,400);
            sleep(200);
            driveforTime(-.3, 1000);
            sleep(200);
            flipIn();




            ////////////////////
            //FIRST GLYPH PLACED!
            ////////////////////

            //GO IN FOR SECOND GLYPH (☞ﾟ∀ﾟ)☞
            NomNomNom.setPower(-.8);
            sleep(300);
            nomDriveForTime(.6, 1500);
            sleep(100);
            nom();
            sleep(600);
            driveforTime(-.4, 2000);
            sleep(100);
            driveforTime(.2, 1000);
            sleep(100);
            driveforTime(-.4, 700);
            //Copped first glyph?? ⚆ _ ⚆
            stopNom();

            //get into position for second placement
            strafeforTime(0.6, 800);

            //SERVO flip out SEQUENCE
            driveforTime(-.4, 1000);        //align with box .....?
            sleep(300);
            driveforTime(.4, 500);
            sleep(200);

            flipOut();
            sleep(500);
            flipIn();
            sleep(1000);
            driveforTime(-.3, 600);
            sleep(200);
            driveforTime(.4, 500);
            sleep(1000);
            driveforTime(-.3, 800);
            NomNomNom.setPower(-1);
            sleep(200);
            strafeforTime(-5,200);
            sleep(200);
            driveforTime(-.3, 500);
            sleep(200);
            stopNom();

            //release!
            driveforTime(-.6, 600);


            break;





        }
    }
}
