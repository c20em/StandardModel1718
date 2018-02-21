package org.firstinspires.ftc.teamcode.newomnomnomnomnom;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by student on 2/16/18.
 */

@Autonomous(name="Red Rectangle Corner", group="Autonomous")

public class RedRectangleCorner extends BaseAutoFunctions {

    @Override
    public void runOpMode() throws InterruptedException {
        declare();
        waitForStart();

        liftIn.setPosition(.9);             //Relic Blocker
        elbowServo.setPosition(.2);         //Relic arm up
        delay(1000);                         //breif pause so that wall servo does not interfere with relic arm release
        wallServo.setPosition(.3);          //Wall servo out

        while(opModeIsActive()) {
            telemetry.addLine("made it this far");
            telemetry.update();

            //jewel(false); /////TAKE THIS OUT WHEN USING THE JEWEL
            //sleep(1000);   /////THIS IS JUST WHILE JEWEL SERVO IS UNDER REPAIR
//////////
//            drive(-.8, 300);
//            delay(500);
//            strafe(-.95, 650);
//            delay(500);
//            drive(-.4, 1000);
//            sleep(500);
//            drive(.4, 400);
//            delay(500);
//            flipIn();               //NECESSARY: need the sleep in between for 1000ms to allow servo to actually get to position before being called back in
//            delay(500);
//            flipOut();
//            delay(500);
//            strafe(-.95, 200);
//            drive(-.3, 600);
//
//
//            //FIRST GLYPH PLACED
/////////////
//            strafe(-1, 500);
//            nom(50);
//            delay(200);
//
//            drive(-.2, 1500);//align with wall
//            drive(-.6, 800);//align with wall
//
//            delay(1000);
//            drive(.6,500);
//            strafe(1,200);
//            //second glyph?
//            drive(.3, 1600);
//            nom(500);
//            nomDrive(-.2, 2000);
//            nom(400);
//            //third glyph?
//            drive(.3, 1600);
//            nom(500);
//            nomDrive(-.2, 2000);
//            nom(400);
//
//
//            strafe(1, 800);
//            delay(500);
//            drive(-.4, 1400);
//            delay(500);
//            drive(.4, 400);
//            delay(500);
//            flipIn();               //NECESSARY: need the sleep in between for 1000ms to allow servo to actually get to position before being called back in
//            delay(1000);
//            flipOut();
//            delay(1000);
//            strafe(-.95, 200);
//            drive(-.3, 600);
//            delay(100);
//            drive(.9, 100);
//            delay(100);
//            drive(-.3, 600);
//            delay(100);
//            drive(.9, 100);


            break;
//

        }
    }
}
