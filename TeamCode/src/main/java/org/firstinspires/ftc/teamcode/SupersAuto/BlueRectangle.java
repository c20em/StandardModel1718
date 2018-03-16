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
        initGyro();
        waitForStart();
        initVuforia();
        setStartAngle();
        servoStartSequence();

        while(opModeIsActive()) {

            RelicRecoveryVuMark column = getPicto();
            //Run Jewel sequence
            jewelSequence(true);

            //Drive to Cryptobox
            driveforTime(.6, 800);
            sleep(40);
            specialTurnAngleCCW(currentAngle()-(veryStartAngle-180));
            sleep(40);
            driveforTime(.6,100);
            sleep(40);
            strafeforTime(-.8, 410);

            //Turn and drive to correct column
            turnToColumnSequence(column,0, 180);

            //SERVO flip out SEQUENCE
            placeGlyphSequence();

            returntoCenterSequence(column, 180);

            //GO IN FOR SECOND GLYPH (☞ﾟ∀ﾟ)☞
            getNewGlyphRectangleSequence(-1,180, true);
            // ⚆ _ ⚆

            //get into position for second placement
            turnToSecondColumnSequence(column, false,180);

            //SERVO flip out SEQUENCE
            placeSpaciousGlyphSequence();

            //move back out
            nomDriveForTime(.4, 200);

            break;
        }
    }
}
