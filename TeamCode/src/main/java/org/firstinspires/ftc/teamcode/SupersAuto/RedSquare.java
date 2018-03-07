package org.firstinspires.ftc.teamcode.SupersAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by student on 2/16/18.
 */

@Autonomous(name="Red Square", group="Autonomous")
public class RedSquare extends SupersBaseFunctions {

    @Override
    public void runOpMode() throws InterruptedException {

        declare();
        initVuforia();
        initGyro();
        waitForStart();

        setStartAngle();
        servoStartSequence();

        while(opModeIsActive()) {
            telemetry.addLine("made it this far");
            telemetry.update();

            RelicRecoveryVuMark column = getPicto();

            //Run Jewel sequence
            jewelSequence(false);

            ////////////////////
            //GO FOR FIRST GLYPH
            ////////////////////

            //Drive to Cryptobox
            driveforTime(-0.5, 1150);

            //Turn and drive to correct column
            turnToColumnSequence(column);

            //SERVO flip out SEQUENCE
            placeGlyphSequence();

            returntoCenterSequence(column);
            ////////////////////
            //FIRST GLYPH PLACED!
            ////////////////////

            //GO IN FOR SECOND GLYPH (☞ﾟ∀ﾟ)☞
            getNewGlyphSquareSequence();
            // ⚆ _ ⚆

            //get into position for second placement
            turnToSecondColumnSequence(column);

            //SERVO flip out SEQUENCE
            placeGlyphJankSequence();

            //move back out
            nomDriveForTime(.4, 200);

            break;
        }
    }
}
