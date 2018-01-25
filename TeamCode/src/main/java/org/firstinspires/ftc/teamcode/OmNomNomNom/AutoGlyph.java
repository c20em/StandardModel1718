package org.firstinspires.ftc.teamcode.OmNomNomNom;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Skipper.BlueLeftAuto;

/**
 * Created by student on 1/14/18.
 */
@Autonomous(name="Basic Glyph", group="Autonomous")
public class AutoGlyph extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FrontLeftDrive = null;
    private DcMotor FrontRightDrive = null;
    private DcMotor BackLeftDrive = null;
    private DcMotor BackRightDrive = null;
    private DcMotor lift = null;
    private DcMotor NomNomNom = null;
    private Servo rightBoxServo = null;
    private Servo leftBoxServo = null;

    BaseChassis.boxPosition boxPos = BaseChassis.boxPosition.DOWN;

    @Override public void runOpMode() {

        FrontLeftDrive = hardwareMap.get(DcMotor.class, "front_left");
        FrontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        BackLeftDrive = hardwareMap.get(DcMotor.class, "back_left");
        BackRightDrive = hardwareMap.get(DcMotor.class, "back_right");
        lift = hardwareMap.get(DcMotor.class, "lift");
        NomNomNom = hardwareMap.get(DcMotor.class, "nom");
        rightBoxServo = hardwareMap.get(Servo.class, "right_box_servo");
        leftBoxServo = hardwareMap.get(Servo.class, "left_box_servo");



        FrontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        BackLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        BackRightDrive.setDirection(DcMotor.Direction.FORWARD);
        FrontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);
        NomNomNom.setDirection(DcMotor.Direction.FORWARD);

        FrontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        NomNomNom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        FrontLeftDrive.setPower(0);
        BackLeftDrive.setPower(0);
        BackRightDrive.setPower(0);
        FrontRightDrive.setPower(0);

        waitForStart();

        while(opModeIsActive()) {

            FrontLeftDrive.setPower(.6);
            BackLeftDrive.setPower(.6);
            BackRightDrive.setPower(.6);
            FrontRightDrive.setPower(.6);

            sleep(1200);

            FrontLeftDrive.setPower(0);
            BackLeftDrive.setPower(0);
            BackRightDrive.setPower(0);
            FrontRightDrive.setPower(0);

            NomNomNom.setPower(-1);

            sleep(750);

            NomNomNom.setPower(0);

            FrontLeftDrive.setPower(.6);
            BackLeftDrive.setPower(.6);
            BackRightDrive.setPower(.6);
            FrontRightDrive.setPower(.6);

            sleep(200);

            FrontLeftDrive.setPower(-.6);
            BackLeftDrive.setPower(-.6);
            BackRightDrive.setPower(-.6);
            FrontRightDrive.setPower(-.6);

            sleep(400);

            FrontLeftDrive.setPower(.6);
            BackLeftDrive.setPower(.6);
            BackRightDrive.setPower(.6);
            FrontRightDrive.setPower(.6);

            sleep(400);

            FrontLeftDrive.setPower(-.6);
            BackLeftDrive.setPower(-.6);
            BackRightDrive.setPower(-.6);
            FrontRightDrive.setPower(-.6);

            sleep(400);

            FrontLeftDrive.setPower(0);
            BackLeftDrive.setPower(0);
            BackRightDrive.setPower(0);
            FrontRightDrive.setPower(0);

        }
        telemetry.addData("Running", "False");
        telemetry.update();
    }

    public enum boxPosition {
        HALF_UP, UP, DOWN;
    }

}
