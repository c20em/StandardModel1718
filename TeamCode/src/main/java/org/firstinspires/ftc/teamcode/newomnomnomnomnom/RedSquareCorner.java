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
//
//            jewel(false); /////TAKE THIS OUT WHEN USING THE JEWEL
//            sleep(1000);   /////THIS IS JUST WHILE JEWEL SERVO IS UNDER REPAIR

//            DriveBackwardwithEncoders(.4, 500,false);
//
//            sleep(1000);
//            StrafeRightwithEncoders(.8, 200, false);
//            flipIn();               //NECESSARY: need the sleep in between for 1000ms to allow servo to actually get to position before being called back in
//            sleep(1000);
//            flipOut();
//            sleep(1000);
//            break;

            DriveBackwardwithEncoders(.4, 200,false);
             sleep(1000);
            DriveForwardwithEncoders(.4,200,false);
            sleep(1000);
            StrafeRightwithEncoders(.4, 200, false);
            sleep(1000);
            StrafeLeftwithEncoders(.4, 200, false);
            sleep(1000);
            TurnLeftwithEncoders(.4,200,false);
            sleep(1000);
            TurnRightwithEncoders(.4,200,false);
            break;





        }
    }
}
