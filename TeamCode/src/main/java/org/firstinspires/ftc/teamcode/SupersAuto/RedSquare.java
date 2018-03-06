package org.firstinspires.ftc.teamcode.SupersAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by student on 2/16/18.
 */

@Autonomous(name="Red Square", group="Autonomous")
public class RedSquare extends SupersBaseFunctions {

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

            getPicto();

            jewel(false);
            sleep(600);

            ////////////////////
            //GO FOR FIRST GLYPH
            ////////////////////

            //Drive to Cryptobox
            driveforTime(-.5, 1100);
            sleep(200);
            turn(0.8, 480);
            sleep(300);

            //SERVO flip out SEQUENCE
            driveforTime(-.4, 500);        //align with box .....?
            sleep(300);
            driveforTime(.4, 300);
            sleep(200);

            flipOut();
            sleep(1000);
            driveforTime(-.3, 400);
            sleep(200);
            driveforTime(.4, 400);
            sleep(200);
            flipIn();
            sleep(1000);
            driveforTime(-.3, 500);
            sleep(300);
            strafeforTime(-.5,300);
            sleep(200);
            driveforTime(-.3, 500);
            sleep(200);



            ////////////////////
            //FIRST GLYPH PLACED!
            ////////////////////

            //GO IN FOR SECOND GLYPH (☞ﾟ∀ﾟ)☞
            NomNomNom.setPower(-1);
            sleep(400);
            nomDriveForTime(.7, 1500);
            sleep(100);
            nom();
            sleep(300);
            nomDriveForTime(-.7, 800);
            sleep(100);
            nom();
            sleep(300);
            nomDriveForTime(.3, 800);
            nom();
            sleep(300);
            nomDriveForTime(-.4, 1000);
            //Copped first glyph?? ⚆ _ ⚆

            //get into position for second placement
            strafeforTime(.8, 400);

            //SERVO flip out SEQUENCE
            driveforTime(-.4, 500);        //align with box .....?
            sleep(300);
            driveforTime(.4, 300);
            sleep(200);

            flipOut();
            sleep(1000);
            driveforTime(-.3, 400);
            sleep(200);
            driveforTime(.4, 400);
            sleep(200);
            flipIn();
            sleep(1000);
            driveforTime(-.3, 500);
            NomNomNom.setPower(-1);
            sleep(200);
            strafeforTime(-5,100);
            sleep(200);
            driveforTime(-.3, 500);
            sleep(200);

            //release!
            driveforTime(.3, 200);


            break;





        }
    }
}
