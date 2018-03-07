/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.ArchAuto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name="Glyph Right Blue", group ="Glyph")
@Disabled
public class BlueRightGlyph extends LinearOpMode {

    public ColorSensor colorSensorL;
    public Servo loweringJewelServo;
    public Servo turningJewelServo;

    Servo leftTop;
    Servo rightTop;
    Servo rightBottom;
    Servo leftBottom;

    private DcMotor FrontLeftDrive = null;
    private DcMotor FrontRightDrive = null;
    private DcMotor BackLeftDrive = null;
    private DcMotor BackRightDrive = null;

    private DcMotor LiftDrive = null;


    static final double CLOSE_TOP_LEFT = 0.38;
    static final double CLOSE_TOP_RIGHT = 0.62;
    static final double OPEN_TOP_LEFT     =  0.95;
    static final double OPEN_TOP_RIGHT     =  0.05;

    static final double SEMI_OPEN_BOTTOM_RIGHT = 0.65;
    static final double SEMI_OPEN_BOTTOM_LEFT = 0.3;
    static final double SEMI_OPEN_TOP_RIGHT = 0.15;
    static final double SEMI_OPEN_TOP_LEFT = 0.48;

    public double downPos = .85;
    public final double UP_POS = 0.3;

    public final double LEFT_POS = .30;
    public final double RIGHT_POS = .70;

    public final double MIDDLE_POS = .5;

    public double increment = .07;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    @Override public void runOpMode() {
        rightTop = hardwareMap.get(Servo.class, "right top claw");
        leftTop = hardwareMap.get(Servo.class, "left top claw");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        colorSensorL = hardwareMap.get(ColorSensor.class, "color sensor left");
        loweringJewelServo = hardwareMap.get(Servo.class, "lowering servo" );
        turningJewelServo = hardwareMap.get(Servo.class, "turning servo");

        FrontLeftDrive = hardwareMap.get(DcMotor.class, "front_left");
        FrontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        BackLeftDrive = hardwareMap.get(DcMotor.class, "back_left");
        BackRightDrive = hardwareMap.get(DcMotor.class, "back_right");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        FrontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        BackLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        BackRightDrive.setDirection(DcMotor.Direction.FORWARD);
        FrontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        LiftDrive.setDirection(DcMotor.Direction.FORWARD);

        FrontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        loweringJewelServo.setPosition(0);
        turningJewelServo.setPosition(.5);

        FrontLeftDrive.setPower(0);
        BackLeftDrive.setPower(0);
        BackRightDrive.setPower(0);
        FrontRightDrive.setPower(0);

        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        lower();
        close();
        lift();


        while(opModeIsActive()) {
            telemetry.addData("Turing Servo:", turningJewelServo.getPosition());
            sleep(1000);
            blue();

            sleep(1500);

            FrontLeftDrive.setPower(.6);
            BackLeftDrive.setPower(.6);
            BackRightDrive.setPower(.5);
            FrontRightDrive.setPower(.5);

            sleep(500);

            turnToAngle(90);

            sleep(600);

            FrontLeftDrive.setPower(.5);
            BackLeftDrive.setPower(.5);
            BackRightDrive.setPower(.5);
            FrontRightDrive.setPower(.5);

            sleep(200);

            FrontLeftDrive.setPower(0);
            BackLeftDrive.setPower(0);
            BackRightDrive.setPower(0);
            FrontRightDrive.setPower(0);
        }
        telemetry.addData("Running", "False");
        telemetry.update();
    }

    public boolean isLeft() {

        telemetry.addData("Red:", colorSensorL.red());
        telemetry.addData("Blue:", colorSensorL.blue());

        telemetry.update();

        if (colorSensorL.red() > colorSensorL.blue()) {
            telemetry.addLine("See Red");
            telemetry.update();
            return true;
        } else {
            telemetry.addLine("See Blue");
            telemetry.update();
            return false;
        }
    }

    public void lower() {

        loweringJewelServo.setPosition(downPos);
        telemetry.addData("lowerArm", loweringJewelServo.getPosition());
        telemetry.update();
    }



    public void blue() {
        telemetry.addData("Red:", colorSensorL.red());
        telemetry.addData("Blue:", colorSensorL.blue());

        telemetry.update();

        if (colorSensorL.red() < colorSensorL.blue()) {
            turningJewelServo.setPosition(RIGHT_POS);
            telemetry.addLine("Moving Right");

            sleep(1000);
            loweringJewelServo.setPosition(.4);
            turningJewelServo.setPosition(.5);
            loweringJewelServo.setPosition(0);
        }
        else if (colorSensorL.red() > colorSensorL.blue()){
            turningJewelServo.setPosition(LEFT_POS);
            telemetry.addLine("Hitting Left");

            sleep(1000);
            loweringJewelServo.setPosition(.4);
            turningJewelServo.setPosition(.5);
            loweringJewelServo.setPosition(0);
        } else {
            turningJewelServo.setPosition(.46);
            loweringJewelServo.setPosition(.95);
            sleep(1000);

            if (colorSensorL.red() > colorSensorL.blue()) {
                turningJewelServo.setPosition(RIGHT_POS);
                telemetry.addLine("Moving Right");

                sleep(1000);
                loweringJewelServo.setPosition(0);
                turningJewelServo.setPosition(.5);
            }
            else if (colorSensorL.red() < colorSensorL.blue()){
                turningJewelServo.setPosition(LEFT_POS);
                telemetry.addLine("Hitting Left");

                sleep(1000);
                loweringJewelServo.setPosition(0);
                turningJewelServo.setPosition(.5);
            }
            else {
                loweringJewelServo.setPosition(.4);
                turningJewelServo.setPosition(.5);
                loweringJewelServo.setPosition(0);
            }
        }
//
        telemetry.addData("Servo Pos", turningJewelServo.getPosition());
        telemetry.update();
    }

    public void close() {
        rightTop.setPosition(CLOSE_TOP_RIGHT);
        leftTop.setPosition(CLOSE_TOP_LEFT);
    }

    public void open() {
        rightTop.setPosition(OPEN_TOP_RIGHT);
        leftTop.setPosition(OPEN_TOP_LEFT);
    }

    public void lift() {
        LiftDrive.setPower(.5);
        sleep(400);
    }

    public void spinClockwise(double power) {
        FrontLeftDrive.setPower(power);
        BackLeftDrive.setPower(power);
        BackRightDrive.setPower(-power);
        FrontRightDrive.setPower(-power);
    }

    public void spinCounter(double power) {
        FrontLeftDrive.setPower(-power);
        BackLeftDrive.setPower(-power);
        BackRightDrive.setPower(power);
        FrontRightDrive.setPower(power);
    }

    public void turnToAngle(float angle) {
        float current;

        angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        current = angles.firstAngle;

        if (angle > current) {
            while (angle >= current + 5 && angle <= current -5) {
                spinClockwise(.5);
                current = angles.firstAngle;
            }
        } else {
            while (angle >= current + 5 && angle <= current -5) {
                spinCounter(.5);
                current = angles.firstAngle;
            }
        }
    }
}
