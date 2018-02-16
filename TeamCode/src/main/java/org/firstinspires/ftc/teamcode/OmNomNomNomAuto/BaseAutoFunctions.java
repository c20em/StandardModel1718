package org.firstinspires.ftc.teamcode.OmNomNomNomAuto;

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



@Autonomous(name="Base Functions", group ="Autonomous")

public class BaseAutoFunctions extends LinearOpMode {
    private DcMotor FrontLeftDrive = null;
    private DcMotor FrontRightDrive = null;
    private DcMotor BackLeftDrive = null;
    private DcMotor BackRightDrive = null;
    private Servo jewelServo;
    public ColorSensor colorSensor;

    static final double JEWEL_DOWN_POS = 0.2;
    static final double JEWEL_UP_POS = 0.5;

    double strafepower = 0.85;
    double drivepower = .3;


    @Override
    public void runOpMode() throws InterruptedException {
        FrontLeftDrive = hardwareMap.get(DcMotor.class, "front_left");
        FrontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        BackLeftDrive = hardwareMap.get(DcMotor.class, "back_left");
        BackRightDrive = hardwareMap.get(DcMotor.class, "back_right");
        jewelServo = hardwareMap.get(Servo.class, "jewel_servo");
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");

        while (opModeIsActive()) {

        }

    }

    //STRAFING CONTROL
    public void Strafe(int strafedirection) {

        double FRpower = strafedirection * strafepower;
        double BLpower = strafedirection * strafepower;
        double BRpower = -1 * strafedirection * strafepower;
        double FLpower = -1 * strafedirection * strafepower;

        FLpower = Range.clip(FLpower, -1.0, 1.0);
        BRpower = Range.clip(BRpower, -1.0, 1.0);
        BLpower = Range.clip(BLpower, -1.0, 1.0);
        FRpower = Range.clip(FRpower, -1.0, 1.0);

        readjustMotorPower(FRpower);
        readjustMotorPower(BRpower);
        readjustMotorPower(FLpower);
        readjustMotorPower(BLpower);

        FrontLeftDrive.setPower(FLpower);
        BackLeftDrive.setPower(BLpower);
        FrontRightDrive.setPower(FRpower);
        BackRightDrive.setPower(BRpower);

    }

    //DRIVING FOWARADS/BACKWARDS/TURNING
    public void Drive(double drive) {
        double drivePower = drive;

        drivePower = readjustMotorPower(drivePower);
        drivePower = Range.clip(drivePower, -1.0, 1.0);

        FrontLeftDrive.setPower(drivePower);
        BackLeftDrive.setPower(drivePower);
        FrontRightDrive.setPower(drivePower);
        BackRightDrive.setPower(drivePower);

        telemetry.addData("Motors", "drive power (%.2f)", drivePower);
    }

    public void turn(double turn) {
        double Rpower = turn;
        double Lpower = -turn;

        Rpower = readjustMotorPower(Rpower);
        Lpower = readjustMotorPower(Lpower);

        Rpower = Range.clip(Rpower, -1.0, 1.0);
        Lpower = Range.clip(Lpower, -1.0, 1.0);

        FrontLeftDrive.setPower(Lpower);
        BackLeftDrive.setPower(Lpower);
        FrontRightDrive.setPower(Rpower);
        BackRightDrive.setPower(Rpower);
    }

    //KEEPS MOTORS FROM STALLING
    public double readjustMotorPower(double motorPower) {
        if (Math.abs(motorPower) >= 0.3) {
            return motorPower;
        } else {
            return 0;
        }
    }

    public void jewel(boolean blue) {
        jewelServo.setPosition(JEWEL_DOWN_POS);
        if(blue && isBlue() || !blue && !isBlue()) {
            Drive(-drivepower);
        } else if(blue && !isBlue() || !blue && isBlue()) {
            Drive(drivepower);
        }
        sleep(200);
    }

    public boolean isBlue() {
        telemetry.addData("Red:", colorSensor.red());
        telemetry.addData("Blue:", colorSensor.blue());

        telemetry.update();

        if (colorSensor.red() < colorSensor.blue()) {
            return true;
        } else {
            return false;
        }
    }
}