package org.firstinspires.ftc.teamcode.OmNomNomNom;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Libby on 12/14/17.
 */

@TeleOp(name="Om Nom Base", group="Linear Opmode")
public class BaseChassis extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FrontLeftDrive = null;
    private DcMotor FrontRightDrive = null;
    private DcMotor BackLeftDrive = null;
    private DcMotor BackRightDrive = null;
    private DcMotor lift = null;
    private DcMotor NomNomNom = null;
    private Servo rightBoxServo = null;
    private Servo leftBoxServo = null;



    //static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle

    static final double NOM_FORWARD_POWER = -1;
    static final double NOM_BACKWARD_POWER = 1;

    // Define class members
    double strafepower = 1;

    controllerPos previousDrive = controllerPos.ZERO;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

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

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {
            telemetry.addData("x stick", gamepad1.left_stick_x);
            telemetry.addData("y stick", gamepad1.left_stick_y);
            Nom();
            flip();
            moveRobot();
            moveLift();
            telemetry.update();
            idle();
        }
    }
    public enum controllerPos {
        STRAFE_RIGHT, STRAFE_LEFT, DRIVE_FOWARD, DRIVE_BACK, TURN_RIGHT, TURN_LEFT, SLOW_MODE, ZERO;
    }

    //DRIVING CONTROL
    public void moveRobot() {
        double drive = -gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x;

        if(drive > 0.25 && (previousDrive == controllerPos.DRIVE_FOWARD || previousDrive == controllerPos.ZERO)) {
            previousDrive = controllerPos.DRIVE_FOWARD;
            Drive(drive);
        } else if(drive < -0.25 && (previousDrive == controllerPos.DRIVE_BACK || previousDrive == controllerPos.ZERO)) {
            previousDrive = controllerPos.DRIVE_BACK;
            Drive(drive);
        } else if(gamepad1.dpad_right && (previousDrive == controllerPos.STRAFE_RIGHT || previousDrive == controllerPos.ZERO)) {
            previousDrive = controllerPos.STRAFE_RIGHT;
            Strafe(-1);
        } else if(gamepad1.dpad_left && (previousDrive == controllerPos.STRAFE_LEFT || previousDrive == controllerPos.ZERO)) {
            previousDrive = controllerPos.STRAFE_LEFT;
            Strafe(1);
        }  else if(turn > 0.25 &&(previousDrive == controllerPos.TURN_RIGHT || previousDrive == controllerPos.ZERO)){
            previousDrive = controllerPos.TURN_RIGHT;
            turn(turn);
        } else if(turn < -0.25 &&(previousDrive == controllerPos.TURN_LEFT || previousDrive == controllerPos.ZERO)){
            previousDrive = controllerPos.TURN_LEFT;
            turn(turn);
        }
        else {
            previousDrive = controllerPos.ZERO;
            FrontLeftDrive.setPower(0);
            BackLeftDrive.setPower(0);
            FrontRightDrive.setPower(0);
            BackRightDrive.setPower(0);
        }

    }

    //STRAFING CONTROL
    public void Strafe(int strafedirection) {

        double FRpower = strafedirection * strafepower;
        double BLpower = strafedirection * strafepower;
        double BRpower = -1 * strafedirection * strafepower;
        double FLpower = -1 * strafedirection * strafepower ;

        FLpower = Range.clip(FLpower, -1.0, 1.0) ;
        BRpower = Range.clip(BRpower, -1.0, 1.0) ;
        BLpower = Range.clip(BLpower, -1.0, 1.0) ;
        FRpower = Range.clip(FRpower, -1.0, 1.0) ;

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
    public void turn(double turn){
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
    public void flip () {
        telemetry.addLine("Box Servo Left: " + leftBoxServo.getPosition());
        telemetry.addLine("Box Servo Right: " + rightBoxServo.getPosition());
        if(gamepad2.a){
            leftBoxServo.setPosition(.75);
            rightBoxServo.setPosition(.2);
        } else if(gamepad2.y){
            leftBoxServo.setPosition(.1);
            rightBoxServo.setPosition(.85);
        } else if(gamepad2.x &&  leftBoxServo.getPosition() >.1 && rightBoxServo.getPosition() <.85){
            leftBoxServo.setPosition(leftBoxServo.getPosition()-.01);
            rightBoxServo.setPosition(rightBoxServo.getPosition()+.01);
        } else if(gamepad2.b &&  leftBoxServo.getPosition() <.75 && rightBoxServo.getPosition() >.2){
            leftBoxServo.setPosition(leftBoxServo.getPosition()+.01);
            rightBoxServo.setPosition(rightBoxServo.getPosition()-.01);
        }
    }


    public void moveLift() {
        double LiftPower;
        if(gamepad2.left_bumper) {
            LiftPower = 1;
        } else if (gamepad2.right_bumper) {
            LiftPower = -1;
        } else {
            LiftPower = 0;
        }
        lift.setPower(LiftPower);
    }

    public void Nom() {
        double nomfoward = gamepad1.right_trigger;
        double nombackward = gamepad1.left_trigger;
        nomfoward = Range.clip(nomfoward, 0, 1);
        nombackward = Range.clip(nombackward, 0, 1);

        if(nomfoward > .2) {
            NomNomNom.setPower(NOM_FORWARD_POWER);
            telemetry.addLine("nomnomfoward" + nomfoward);
        }
        else if(nombackward > 0.2) {
            NomNomNom.setPower(NOM_BACKWARD_POWER);
            telemetry.addLine("nomnombackward" + nombackward);
        }
        else {
            NomNomNom.setPower(0);
        }
    }

    //KEEPS MOTORS FROM STALLING
    public double readjustMotorPower(double motorPower) {
        if (Math.abs(motorPower) >= 0.3) {
            return motorPower;
        } else {
            return 0;
        }
    }
}




