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
        delay(1000);                         //breif pause so that wall servo does not interfere with relic arm release
        wallServo.setPosition(.3);          //Wall servo out

        while(opModeIsActive()) {
            telemetry.addLine("made it this far");
            telemetry.update();

            //jewel(true); /////TAKE THIS OUT WHEN USING THE JEWEL
            //sleep(1000);   /////THIS IS JUST WHILE JEWEL SERVO IS UNDER REPAIR

            lift(.5, 400);
            delay(200);
            drive(-.4, 500);
            delay(200);
            lift(-.5, 400);
            delay(500);
            turn(-.5, 650);
            delay(500);
            drive(-.4, 800);
            delay(500);
            drive(.4, 400);

            flipIn();               //NECESSARY: need the sleep in between for 1000ms to allow servo to actually get to position before being called back in
            delay(1000);
            flipOut();
            delay(1000);

            drive(-.3, 1000);
            delay(500);
            strafe(-1, 300);
            delay(500);
            drive(-.3, 1000);
            delay(500);
            telemetry.addLine("going for 2");
            telemetry.update();
            //BEGIN 2nd GLYPH!!

            delay(500);
            nomDrive(.7,400);
            delay(100);
            drive(-.7, 400);
            delay(500);
            drive(.6, 1500);
            telemetry.addLine("drive forward with nom");
            telemetry.update();
            nom(400);

            nomDrive(-.4, 1000);

            nom(600);

            nomDrive(.6, 400);

            nom(400);

            nomDrive(-.4, 800);

            delay(500);
            strafe(.75, 300);

            drive(.4, 400);

            flipIn();               //NECESSARY: need the sleep in between for 1000ms to allow servo to actually get to position before being called back in
            delay(1000);
            flipOut();
            delay(1000);
            drive(.3, 1000);
            delay(500);
            strafe(-1, 800);
            delay(500);
            drive(-.3, 1000);
            delay(500);
            break;
//

        }
    }
}
