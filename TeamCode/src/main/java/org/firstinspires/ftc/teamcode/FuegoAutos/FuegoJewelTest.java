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

@Autonomous(name="Jewel Test Fuego", group="Autonomous")

public class FuegoJewelTest extends LinearOpMode {
    FuegoRobot omnom;
    @Override
    public void runOpMode() throws InterruptedException {
        omnom = new FuegoRobot();
        omnom.init(this);
        waitForStart();
        omnom.initPositions();
        while(opModeIsActive()) {
            telemetry.addLine("made it this far");
            telemetry.update();

            //run jewel stuff
                omnom.jewel(false);

        }
    }
}
