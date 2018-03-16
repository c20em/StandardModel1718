package org.firstinspires.ftc.teamcode.SupersAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by student on 2/16/18.
 */

@Autonomous(name="Blue Square", group="Autonomous")
public class BlueSquare extends SupersBaseFunctions {

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
            jewelSequence(true);

            //Drive to Cryptobox
            driveforTime(0.5, 1350);

            //Turn and drive to correct column
            turnToColumnAbbySequence(column, 90);

            //SERVO flip out SEQUENCE
            placeGlyphSequence();

            returntoCenterSequence(column,90);

            //GO IN FOR SECOND GLYPH (☞ﾟ∀ﾟ)☞
            getNewGlyphSquareSequence();
            // ⚆ _ ⚆

            strafeforTime(.3,300);


            //get into position for second placement
            turnToSecondColumnAbbySequence(column, 90);

            //SERVO flip out SEQUENCE
            placeGlyphSequence();

            driveforTime(.8,200);

            break;
        }
    }
}
