package org.firstinspires.ftc.teamcode.newomnomnomnomnom;

/**
 * Created by student on 2/15/18.
 */
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.nio.channels.FileLockInterruptionException;

abstract class BaseAutoFunctions extends LinearOpMode {
    DcMotor FrontLeftDrive = null;
    DcMotor FrontRightDrive = null;
    DcMotor BackLeftDrive = null;
    DcMotor BackRightDrive = null;
    DcMotor lift = null;
    private DcMotor NomNomNom = null;
    private Servo rightBoxServo = null;
    private Servo leftBoxServo = null;


    Servo jewelServo;
    ColorSensor colorSensor;

    static final double JEWEL_DOWN_POS = 0.2;
    static final double JEWEL_UP_POS = 0.5;

    static double BOX_RIGHT_UP = .84;
    static double BOX_LEFT_UP = .1;
    static final double BOX_RIGHT_DOWN = .34;
    static final double BOX_LEFT_DOWN = .61;

    double strafepower = 0.85;
    boolean jewelHit = false;
    double nomPower = .95;

    public void strafeforTime(int strafedirection, long time){

        double FRpower = strafedirection * strafepower;
        double BLpower = strafedirection * strafepower;
        double BRpower = -1 * strafedirection * strafepower;
        double FLpower = -1 * strafedirection * strafepower;

        FrontLeftDrive.setPower(FLpower);
        BackLeftDrive.setPower(BLpower);
        FrontRightDrive.setPower(FRpower);
        BackRightDrive.setPower(BRpower);
        sleep(time);

    }

    public void turnforTime(double turn, long time){

        FrontLeftDrive.setPower(turn);
        BackLeftDrive.setPower(turn);
        FrontRightDrive.setPower(-turn);
        BackRightDrive.setPower(-turn);
        sleep(time);
    }
    public void driveforTime(double drivepower, long time){
        FrontLeftDrive.setPower(drivepower);
        BackLeftDrive.setPower(drivepower);
        FrontRightDrive.setPower(drivepower);
        BackRightDrive.setPower(drivepower);
        telemetry.addData("Motors", "drive power (%.2f)", drivepower);
        sleep(time);
    }
    public void liftforTime(double liftpower, long time){
        lift.setPower(liftpower);
        sleep(time);
    }
    public void autoFlip(boolean flipTrue){
        leftBoxServo.setPosition(BOX_LEFT_DOWN);
        rightBoxServo.setPosition(BOX_RIGHT_DOWN);
        sleep(20);
        leftBoxServo.setPosition(BOX_LEFT_UP-.1);
        rightBoxServo.setPosition(BOX_RIGHT_UP+.1);
    }
    public void driveBackNomming(double drivepower){
        NomNomNom.setPower(nomPower);
        FrontLeftDrive.setPower(-drivepower/2);
        BackLeftDrive.setPower(-drivepower/2);
        FrontRightDrive.setPower(-drivepower/2);
        BackRightDrive.setPower(-drivepower/2);
        sleep(600);
        NomNomNom.setPower(0);
        sleep(300);
        NomNomNom.setPower(nomPower);
        sleep(500);
    }

    public void jewel(boolean blue){
        servoSequence();
        telemetry.addData("Blue:", colorSensor.blue());
        telemetry.addData("Red:", colorSensor.red());
        telemetry.update();
        double turn = 0;

        sleep(500);
        if(blue && isBlue() || !blue && !isBlue()) {
            telemetry.addLine("Blue!");
            turn = -.2;
        } else if(blue && !isBlue() || !blue && isBlue()) {
            telemetry.addLine("Red!");
            turn = .2;
        }
        turnforTime(turn, 50);
        jewelServo.setPosition(.6);
        turnforTime(-turn, 50);

        jewelHit = true;
    }

    public void servoSequence() {
        jewelServo.setPosition(.45);
        sleep(1600);
        jewelServo.setPosition(.2);
        sleep(800);
    }

    public boolean isBlue() {
        telemetry.addData("Red:", colorSensor.red());
        telemetry.addData("Blue:", colorSensor.blue());

        telemetry.update();
        colorSensor.enableLed(true);
        if (colorSensor.red() < colorSensor.blue()) {
            return true;
        } else {
            return false;
        }
    }
}