package org.firstinspires.ftc.teamcode.OmNomNomNomAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Autonomous(name="Blue Left Glyph", group="Autonomous")
@Disabled
public class BlueLeftGlyph extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor FrontLeftDrive = null;
    private DcMotor FrontRightDrive = null;
    private DcMotor BackLeftDrive = null;
    private DcMotor BackRightDrive = null;
    private DcMotor NomNomNom = null;

    public Servo boxRight = null;
    public Servo boxLeft = null;

    //Define global variables
    static final double NOM_FORWARD_POWER = -.9;
    static final double NOM_BACKWARD_POWER = NOM_FORWARD_POWER * -0.5;
    double servopos = 0.5;


    @Override public void runOpMode() {

        FrontLeftDrive = hardwareMap.get(DcMotor.class, "front_left");
        FrontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        BackLeftDrive = hardwareMap.get(DcMotor.class, "back_left");
        BackRightDrive = hardwareMap.get(DcMotor.class, "back_right");
        NomNomNom = hardwareMap.get(DcMotor.class, "nom");

        FrontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        BackLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        BackRightDrive.setDirection(DcMotor.Direction.FORWARD);
        FrontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        NomNomNom.setDirection(DcMotor.Direction.FORWARD);

        FrontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        NomNomNom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        FrontLeftDrive.setPower(0);
        BackLeftDrive.setPower(0);
        BackRightDrive.setPower(0);
        FrontRightDrive.setPower(0);

        waitForStart();

        while(opModeIsActive()) {

            FrontLeftDrive.setPower(.55);
            BackLeftDrive.setPower(.55);
            BackRightDrive.setPower(.5);
            FrontRightDrive.setPower(.5);

            sleep(800);

            FrontLeftDrive.setPower(.55);
            BackLeftDrive.setPower(-.55);
            BackRightDrive.setPower(.5);
            FrontRightDrive.setPower(-.5);

            sleep(550);

            nom(400);

            FrontLeftDrive.setPower(.55);
            BackLeftDrive.setPower(.55);
            BackRightDrive.setPower(.5);
            FrontRightDrive.setPower(.5);

            sleep(400);

            FrontLeftDrive.setPower(-.55);
            BackLeftDrive.setPower(-.55);
            BackRightDrive.setPower(-.5);
            FrontRightDrive.setPower(-.5);

            sleep(400);

            FrontLeftDrive.setPower(.55);
            BackLeftDrive.setPower(.55);
            BackRightDrive.setPower(.5);
            FrontRightDrive.setPower(.5);

            sleep(700);

            FrontLeftDrive.setPower(-.55);
            BackLeftDrive.setPower(-.55);
            BackRightDrive.setPower(-.5);
            FrontRightDrive.setPower(-.5);

            sleep(400);

            FrontLeftDrive.setPower(0);
            BackLeftDrive.setPower(0);
            BackRightDrive.setPower(0);
            FrontRightDrive.setPower(0);

        }
        telemetry.addData("Running", "False");
        telemetry.update();
    }

    public void nom(int sleepTime){
        NomNomNom.setPower(NOM_BACKWARD_POWER);
        sleep(sleepTime);
        NomNomNom.setPower(0);
    }

}
