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

@Autonomous(name="Red Rectangle Corner", group="Autonomous")
@Disabled
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

            jewel(false); /////TAKE THIS OUT WHEN USING THE JEWEL
            sleep(3000);   /////THIS IS JUST WHILE JEWEL SERVO IS UNDER REPAIR


            driveforTime(-.5, 1000);
            sleep(200);
            strafeforTime(.8, 1000);
            sleep(500);
            driveforTime(.4, 1000);
            sleep(1000);
            flipOut();
            sleep(1000);
            flipIn();
            sleep(1000);
            driveforTime(-.3, 600);





            break;
//

        }
    }
}
