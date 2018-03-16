package org.firstinspires.ftc.teamcode.SupersAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by student on 2/16/18.
 */

@Autonomous(name="Blue Rectangle", group="Autonomous")
public class BlueRectangle extends SupersBaseFunctions {

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
            jewelSequence(true);

            veryStartAngle = (veryStartAngle + 180)%360;

            //Drive to Cryptobox
            driveforTime(.6, 680);
            sleep(100);
            turnAngle(180);
            sleep(50);
            strafeforTime(-.8, 450);

            //Turn and drive to correct column
            turnToColumnSequence(column,0);

            //SERVO flip out SEQUENCE
            placeGlyphSequence();

            returntoCenterSequence(column, false);

            //GO IN FOR SECOND GLYPH (☞ﾟ∀ﾟ)☞
            getNewGlyphRectangleSequence(-1);
            // ⚆ _ ⚆

            //get into position for second placement
            turnToSecondColumnSequence(column, false);

            //SERVO flip out SEQUENCE
            placeSpaciousGlyphSequence();

            //move back out
            nomDriveForTime(.4, 200);

            break;
        }
    }
}
