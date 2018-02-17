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

abstract class BaseAutoFunctions extends LinearOpMode {
    DcMotor FrontLeftDrive = null;
    DcMotor FrontRightDrive = null;
    DcMotor BackLeftDrive = null;
    DcMotor BackRightDrive = null;
    Servo jewelServo;
    ColorSensor colorSensor;

    static final double JEWEL_DOWN_POS = 0.2;
    static final double JEWEL_UP_POS = 0.5;

    double strafepower = 0.85;
    double drivepower = .3;
    boolean jewelHit = false;

    //STRAFING CONTROL
    public void Strafe(int strafedirection) {

        double FRpower = strafedirection * strafepower;
        double BLpower = strafedirection * strafepower;
        double BRpower = -1 * strafedirection * strafepower;
        double FLpower = -1 * strafedirection * strafepower;

        FrontLeftDrive.setPower(FLpower);
        BackLeftDrive.setPower(BLpower);
        FrontRightDrive.setPower(FRpower);
        BackRightDrive.setPower(BRpower);

    }

    //DRIVING FOWARADS/BACKWARDS/TURNING
    public void Drive(double drive) {
        double drivePower = drive;

        FrontLeftDrive.setPower(drivePower);
        BackLeftDrive.setPower(drivePower);
        FrontRightDrive.setPower(drivePower);
        BackRightDrive.setPower(drivePower);

        telemetry.addData("Motors", "drive power (%.2f)", drivePower);
    }

    public void turn(double turn) {
        FrontLeftDrive.setPower(turn);
        BackLeftDrive.setPower(turn);
        FrontRightDrive.setPower(turn);
        BackRightDrive.setPower(turn);
    }

    public void jewel(boolean blue) {
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
        turn(turn);
        sleep(300);
        jewelServo.setPosition(.6);
        turn(-turn);
        sleep(300);

        FrontLeftDrive.setPower(0);
        BackLeftDrive.setPower(0);
        FrontRightDrive.setPower(0);
        BackRightDrive.setPower(0);
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