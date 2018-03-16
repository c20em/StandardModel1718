package org.firstinspires.ftc.teamcode.SupersAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by student on 2/16/18.
 */

@Autonomous(name="Red Rectangle", group="Autonomous")
public class RedRectangle extends SupersBaseFunctions {

    @Override
    public void runOpMode() throws InterruptedException {

        declare();
        initVuforia();
        initGyro();
        waitForStart();

        setStartAngle();
        servoStartSequence();

        while(opModeIsActive()) {

            RelicRecoveryVuMark column = getPicto();

            //Run Jewel sequence
            jewelSequence(false);

            //Drive to Cryptobox
            driveforTime(-.6, 650);
            sleep(100);
            strafeforTime(.8, 450);

            //Turn and drive to correct column
            turnToColumnSequence(column,0,0);

            //SERVO flip out SEQUENCE
            placeGlyphSequence();

            returntoCenterSequence(column,0);

            //GO IN FOR SECOND GLYPH (☞ﾟ∀ﾟ)☞
            getNewGlyphRectangleSequence(1,0,false);
            // ⚆ _ ⚆

            //get into position for second placement
            turnToSecondColumnSequence(column, false,0);

            //SERVO flip out SEQUENCE
            placeSpaciousGlyphSequence();

            //move back out
            driveforTime(.4, 200);

            break;
        }
    }
}
