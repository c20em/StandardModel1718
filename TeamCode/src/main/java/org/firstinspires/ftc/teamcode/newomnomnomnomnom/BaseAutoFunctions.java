package org.firstinspires.ftc.teamcode.newomnomnomnomnom;

/**
 * Created by student on 2/15/18.
 */
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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
    DcMotor NomNomNom = null;
    Servo rightBoxServo = null;
    Servo leftBoxServo = null;
    Servo liftIn = null;
    Servo elbowServo = null;
    Servo wallServo = null;
    CRServo pushBackServoLeft = null;
    CRServo pushBackServoRight = null;


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
    double gradualPower = 0.0;

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////

    ///////////////////////////////TIME BASED DRIVE METHODS/////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////


    public void strafeforTime(double strafedirection, long time){

        double FRpower = 1*strafedirection;
        double BLpower = -1*strafedirection;
        double BRpower = -1*strafedirection;
        double FLpower = 1*strafedirection;

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
        sleep(time);

        FrontLeftDrive.setPower(0);
        BackLeftDrive.setPower(0);
        FrontRightDrive.setPower(0);
        BackRightDrive.setPower(0);
    }

    public void turnforTime(double turn, long time) throws InterruptedException{

        FrontLeftDrive.setPower(turn);
        BackLeftDrive.setPower(turn);
        FrontRightDrive.setPower(turn);
        BackRightDrive.setPower(turn);
        sleep(time);
        FrontLeftDrive.setPower(0);
        BackLeftDrive.setPower(0);
        FrontRightDrive.setPower(0);
        BackRightDrive.setPower(0);
        sleep(time);
    }

    public void driveforTime(double drivepower, long time)throws InterruptedException{
        FrontLeftDrive.setPower(-drivepower); //THIS IS DRIVING FORWARDS
        BackLeftDrive.setPower(-drivepower);
        FrontRightDrive.setPower(drivepower);
        BackRightDrive.setPower(drivepower);
        telemetry.addData("Motors", "drive power (%.2f)", drivepower);
        sleep(time);
        FrontLeftDrive.setPower(0);
        BackLeftDrive.setPower(0);
        FrontRightDrive.setPower(0);
        BackRightDrive.setPower(0);
    }

    public void drivewithNom(double drivepower, long time)throws InterruptedException{
        starsAndNomOn(nomPower);
        FrontLeftDrive.setPower(-drivepower);
        BackLeftDrive.setPower(-drivepower);
        FrontRightDrive.setPower(drivepower);
        BackRightDrive.setPower(drivepower);
        sleep(time);
        FrontLeftDrive.setPower(0);
        BackLeftDrive.setPower(0);
        FrontRightDrive.setPower(0);
        BackRightDrive.setPower(0);
    }




    ////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////

    /////////////////////////////ENCODER BASED DRIVE METHODS////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////

    // will call these in ____withEncoders methods
    public void DriveForward(double drivePower) {
        FrontLeftDrive.setPower(-drivePower);
        BackLeftDrive.setPower(-drivePower);
        FrontRightDrive.setPower(drivePower);
        BackRightDrive.setPower(drivePower);
    }
    public void turn(double turn){
        FrontLeftDrive.setPower(turn);
        BackLeftDrive.setPower(turn);
        FrontRightDrive.setPower(turn);
        BackRightDrive.setPower(turn);
    }
    public void StrafeRight(double drivePower) {
        FrontLeftDrive.setPower(drivePower);
        BackLeftDrive.setPower(-drivePower);
        FrontRightDrive.setPower(drivePower);
        BackRightDrive.setPower(-drivePower);
    }
    public void StopDriving(){
        FrontLeftDrive.setPower(0);
        BackLeftDrive.setPower(0);
        FrontRightDrive.setPower(0);
        BackRightDrive.setPower(0);
    }

    //WITH ENCODER DRIVE METHODS
    public void DriveForwardwithEncoders(double drivePower, int distance, boolean nom){

        // reset the encoders
        FrontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set to the target (distance)
        FrontLeftDrive.setTargetPosition(distance);
        BackLeftDrive.setTargetPosition(distance);
        FrontRightDrive.setTargetPosition(distance);
        BackRightDrive.setTargetPosition(distance);

        // set to RUN_TO_POSITION mode
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        DriveForward(drivePower);

        while (FrontLeftDrive.isBusy() && BackLeftDrive.isBusy() && FrontRightDrive.isBusy() && BackRightDrive.isBusy()){
            // waiting until the target position is reached
            // if asked for nom, run nom while moving
            if(nom)NomNomNom.setPower(nomPower);
        }
        // stop driving
        StopDriving();

        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void DriveBackwardwithEncoders(double drivePower, int distance, boolean nom){
        DriveForwardwithEncoders(-drivePower, distance, nom);
    }

    public void TurnLeftwithEncoders(double drivePower, int distance, boolean nom){
        // reset the encoders
        FrontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set to the target (distance)
        FrontLeftDrive.setTargetPosition(distance);
        BackLeftDrive.setTargetPosition(distance);
        FrontRightDrive.setTargetPosition(distance);
        BackRightDrive.setTargetPosition(distance);

        // set to RUN_TO_POSITION mode
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turn(drivePower);

        while (FrontLeftDrive.isBusy() && BackLeftDrive.isBusy() && FrontRightDrive.isBusy() && BackRightDrive.isBusy()){
            // waiting until the target position is reached
            // if asked for nom, run nom while moving
            if(nom)NomNomNom.setPower(nomPower);
        }
        // stop driving
        StopDriving();
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void TurnRightwithEncoders(double drivePower, int distance, boolean nom){
        TurnLeftwithEncoders(-drivePower, distance, nom);
    }

    public void StrafeRightwithEncoders(double drivePower, int distance, boolean nom){
        // reset the encoders
        FrontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set to the target (distance)
        FrontLeftDrive.setTargetPosition(distance);
        BackLeftDrive.setTargetPosition(distance);
        FrontRightDrive.setTargetPosition(distance);
        BackRightDrive.setTargetPosition(distance);

        // set to RUN_TO_POSITION mode
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        StrafeRight(drivePower);

        while (FrontLeftDrive.isBusy() && BackLeftDrive.isBusy() && FrontRightDrive.isBusy() && BackRightDrive.isBusy()){
            // waiting until the target position is reached
            // if asked for nom, run nom while moving
            if(nom)NomNomNom.setPower(nomPower);
        }
        // stop driving
        StopDriving();

        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void StrafeLeftwithEncoders(double drivePower, int distance, boolean nom) {
        StrafeLeftwithEncoders(-drivePower, distance, nom);
    }

    //prevent motors from stalling out
    public double readjustMotorPower(double motorPower) {
        if (Math.abs(motorPower) >= 0.3) {
            return motorPower;
        } else {
            return 0;
        }
    }

    //time based nom only
    public void nomforTime(double power, long time)throws InterruptedException {
        NomNomNom.setPower(-power);
        sleep(time);
        NomNomNom.setPower(0);
    }

    //time based nom and stars in only
    public void starsAndNomOn(double power) {
        NomNomNom.setPower(-power);
        pushBackServoLeft.setPower(-1);
        pushBackServoRight.setPower(1);
    }

    //time based lift
    public void liftforTime(double liftpower, long time)throws InterruptedException{
        lift.setPower(liftpower);
        sleep(time);
        lift.setPower(0);
    }

    //glyph place flip out
    public void flipOut(){
        leftBoxServo.setPosition(BOX_LEFT_UP);
        rightBoxServo.setPosition(BOX_RIGHT_UP);
    }

    //blyph place flip in
    public void flipIn(){
        leftBoxServo.setPosition(BOX_LEFT_DOWN);
        rightBoxServo.setPosition(BOX_RIGHT_DOWN);
    }

    public void jewel(boolean blue) throws InterruptedException {
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