package org.firstinspires.ftc.teamcode.FuegoAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * peep FuegoRobot.java
 */

@Autonomous(name="Blue 1 Fuego", group="Autonomous")

public class Blue1Fuego extends LinearOpMode {
    FuegoRobot omnom;
    @Override
    public void runOpMode() throws InterruptedException {
        omnom = new FuegoRobot();
        omnom.init(this);
        waitForStart();
        omnom.initPositions();

        telemetry.addLine("made it this far");
        telemetry.update();

        //get vuforia column
        omnom.column = omnom.getPicto();

        //run jewel stuff
        omnom.jewel(true);
        sleep(600);

        //MOVE TO THE CORRECT COLUMN
        if (omnom.column == RelicRecoveryVuMark.CENTER || omnom.column == RelicRecoveryVuMark.UNKNOWN) {
            omnom.driveforTime(0.5, 1200); //fill w center value
        } else if (omnom.column == RelicRecoveryVuMark.LEFT) {
            omnom.driveforTime(0.5, 1200); //fill w left value
        } else if (omnom.column == RelicRecoveryVuMark.RIGHT) {
            omnom.driveforTime(0.5, 1200);//fill w right value
        } else  omnom.driveforTime(0.5, 1200);

        sleep(200);
        omnom.turnAngleCW(90);
        sleep(300);
        omnom.driveforTime(.7, 800);

        //place the first glyph
        omnom.placeGlyphSequence();

    }
}
