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
        wallServo.setPosition(.4);

        while(opModeIsActive()) {
            telemetry.addLine("made it this far");
            telemetry.update();

            RelicRecoveryVuMark column = getPicto();

            //Run Jewel sequence
            jewelSequence(false);

            //Drive to Cryptobox
            driveforTime(-0.5, 1000);

            //Turn and drive to correct column
            turnToColumnAbbySequence(column, 90);

            //SERVO flip out SEQUENCE
            placeGlyphSequence();

            returntoCenterSequence(column, true);

            //GO IN FOR SECOND GLYPH (☞ﾟ∀ﾟ)☞
            getNewGlyphSquareSequence();
            // ⚆ _ ⚆

            strafeforTime(.3,300);

            //get into position for second placement
            //turnToSecondColumnSequence(column, true);
            turnToSecondColumnAbbySequence(column, true, false);

            //SERVO flip out SEQUENCE
            placeGlyphSequence();

            driveforTime(.8,200);

//
//            //move back out
//            nomDriveForTime(.4, 200);

            break;
        }
    }
}
