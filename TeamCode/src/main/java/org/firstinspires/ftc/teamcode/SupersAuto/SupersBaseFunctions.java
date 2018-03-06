package org.firstinspires.ftc.teamcode.SupersAuto;

/**
 * Created by student on 2/15/18.
 */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

abstract class SupersBaseFunctions extends LinearOpMode {

    /*
    initializing motors, servos, and sensors
     */

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
    ColorSensor colorSensor;
    Servo jewelServo;
    Servo jewelSideServo;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    static final double JEWEL_DOWN_POS = 0.0;
    static final double JEWEL_UP_POS = 1;

    static final double SIDE_JEWEL_NEUTRAL_POS = 0.6;

    static double BOX_RIGHT_DOWN = .84;
    static double BOX_LEFT_DOWN = .1;
    static final double BOX_RIGHT_UP = .34;
    static final double BOX_LEFT_UP = .61;
    static final double ELBOW_UP = .1;

    static final double JEWEL_TURNCCW_POS = .68;
    static final double JEWEL_TURNMID_POS = .59;
    static final double JEWEL_TURNCW_POS = .5;

    double nomPower = 0.95;
    boolean canSeeJewel = false;

    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    VuforiaLocalizer.Parameters parameters = null;
    VuforiaTrackables relicTrackables = null;
    VuforiaTrackable relicTemplate = null;

    ElapsedTime clock = new ElapsedTime();

    public void declare() {
        FrontLeftDrive = hardwareMap.get(DcMotor.class, "front_left");
        FrontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        BackLeftDrive = hardwareMap.get(DcMotor.class, "back_left");
        BackRightDrive = hardwareMap.get(DcMotor.class, "back_right");
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        rightBoxServo = hardwareMap.get(Servo.class, "right_box_servo");
        leftBoxServo = hardwareMap.get(Servo.class, "left_box_servo");
        lift = hardwareMap.get(DcMotor.class, "lift");
        NomNomNom = hardwareMap.get(DcMotor.class, "nom");
        liftIn = hardwareMap.get(Servo.class, "lift_in");
        elbowServo = hardwareMap.get(Servo.class, "elbow_servo");
        wallServo = hardwareMap.get(Servo.class, "wall_servo");
        pushBackServoRight = hardwareMap.get(CRServo.class, "push_back_servo_right");
        pushBackServoLeft = hardwareMap.get(CRServo.class, "push_back_servo_left");
        jewelServo = hardwareMap.get(Servo.class, "jewel_servo");
        jewelSideServo = hardwareMap.get(Servo.class, "jewel_side_servo");

        FrontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        BackLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        BackRightDrive.setDirection(DcMotor.Direction.FORWARD);
        FrontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        NomNomNom.setDirection(DcMotor.Direction.REVERSE);

        FrontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        NomNomNom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void initGyro() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void initVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "Aeba4Qn/////AAAAGahNOxzreUE8nItPWzsrOlF7uoyrR/qbjue3kUmhxZcfZMSd5MRyEY+3uEoVA+gpQGz5KyP3wEjBxSOAb4+FBYMZ+QblFU4byMG4+aiI+GeeBA+RatQXVzSduRBdniCW4qehTnwS204KTUMXg1ioPvUlbYQmqM5aPMx/2xnYN1b+htNBWV0Bc8Vkyspa0NNgz7PzF1gozlCIc9FgzbzNYoOMhqSG+jhKf47SZQxD6iEAtj5iAkWBvJwFDLr/EfDfPr3BIA6Cpb4xaDc0t4Iz5wJ/p4oLRiEJaLoE/noCcWFjLmPcw9ccwYXThtjC+7u0DsMX+r+1dMikBCZCWWkLzEyjWzy3pOOR3exNRYGZ0vzr";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTrackables.activate();
    }

    public void strafeforTime(double power, int time) throws InterruptedException {
        strafe(power);
        sleep(time);
        StopDriving();
    }

    public void strafe(double power) {
        FrontLeftDrive.setPower(power);
        BackLeftDrive.setPower(-power);
        FrontRightDrive.setPower(-power);
        BackRightDrive.setPower(power);
    }

    public void turn(double power, int time) throws InterruptedException{
        turn(power);
        sleep(time);
        StopDriving();
    }

    public void driveforTime(double power, int time)throws InterruptedException{
        drive(power);
        sleep(time);
        StopDriving();
    }

    public void drive(double power) {
        FrontLeftDrive.setPower(power); //THIS IS DRIVING BACKWARDS
        BackLeftDrive.setPower(power);
        FrontRightDrive.setPower(power);
        BackRightDrive.setPower(power);
        telemetry.addData("Motors", "drive power (%.2f)", power);
    }

    public void lift(double liftpower, int time) throws InterruptedException{
        lift.setPower(liftpower);
        sleep(time);
        lift.setPower(0);
    }

    public void flipOut(){
        leftBoxServo.setPosition(BOX_LEFT_UP);
        rightBoxServo.setPosition(BOX_RIGHT_UP);
    }
    public void flipIn(){
        leftBoxServo.setPosition(BOX_LEFT_DOWN);
        rightBoxServo.setPosition(BOX_RIGHT_DOWN);
    }

    public void nomforTime(int time) throws InterruptedException {
        nom();
        sleep(time);
        stopNom();
    }

    public void nom() {
        NomNomNom.setPower(nomPower);
        pushBackServoLeft.setPower(-1);
        pushBackServoRight.setPower(1);
    }

    public void stopNom() {
        NomNomNom.setPower(0);
        pushBackServoLeft.setPower(0);
        pushBackServoRight.setPower(0);
    }

    public void nomDrive(double power) {
        nom();
        drive(power);
    }

    public void nomDriveForTime(double power, int time)throws InterruptedException{
        nomDrive(power);
        sleep(time);
        StopDriving();
        stopNom();
    }
    public void jewel(boolean teamBlue) throws InterruptedException {
        boolean jewelBlue;
        jewelSideServo.setPosition(JEWEL_TURNMID_POS);
        sleep(100);
        jewelServo.setPosition(JEWEL_DOWN_POS);
        //read color
        int red = 0;
        int blue = 0;
        for (int i = 0; i < 40; i++) {
            if (colorSensor.red() > colorSensor.blue()) red++;
            if (colorSensor.red() < colorSensor.blue()) blue++;
        }
        telemetry.addLine("read color");

        //decide which color we see
        if(blue>red){
            jewelBlue= true;
            telemetry.addData("blueWins!", blue);
        } else {
            jewelBlue=false;
            telemetry.addData("redWins!", red);
        }
        telemetry.addData("blue: ", blue);
        telemetry.addData("red: ", red);
        telemetry.update();

        //knock off correct jewel
        if(jewelBlue){
            if(teamBlue) jewelSideServo.setPosition(JEWEL_TURNCW_POS);
            if(!teamBlue) jewelSideServo.setPosition(JEWEL_TURNCCW_POS);
        } else if (!jewelBlue){
            if(teamBlue) jewelSideServo.setPosition(JEWEL_TURNCCW_POS);
            if(!teamBlue) jewelSideServo.setPosition(JEWEL_TURNCW_POS);
        }
        sleep(1000);

        //turn back
        jewelSideServo.setPosition(JEWEL_TURNMID_POS);
        sleep(100);
        jewelServo.setPosition(JEWEL_UP_POS);
    }

    public boolean isBlue() throws InterruptedException {
        telemetry.addData("Red:", colorSensor.red());
        telemetry.addData("Blue:", colorSensor.blue());
        telemetry.update();

        colorSensor.enableLed(true);
        sleep(200);
        clock.reset();
        int red = 0;
        int blue = 0;
        while(clock.milliseconds() < 800) {
            red = colorSensor.red();
            blue = colorSensor.blue();
        }
        if (red < blue) {
            telemetry.addLine("SEES BLUE");
            telemetry.update();
            colorSensor.enableLed(false);
            canSeeJewel = true;
            return true;
        } else if(red > blue) {
            telemetry.addLine("SEES RED");
            telemetry.update();
            colorSensor.enableLed(false);
            canSeeJewel = true;
            return false;
        }else{
            canSeeJewel = false;
            return false;
        }
    }

    //      we've got the vision     ( ⚆ _ ⚆ )
    public RelicRecoveryVuMark pictograph() {
        double startTime = getRuntime() * 1000;
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        if(vuMark != RelicRecoveryVuMark.UNKNOWN) {
            return vuMark;
        } else {
            while(startTime - getRuntime() < 2000) {
                vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if(vuMark != RelicRecoveryVuMark.UNKNOWN) {
                    return vuMark;
                }
            }
        }
        return vuMark;
    }


    //DELAY
    public void delay(int milliseconds) throws InterruptedException {
        clock.reset();
        while(clock.milliseconds() < milliseconds) {
            telemetry.addLine("sleepyy");
        }
    }


    public void turn(double turn){
        FrontLeftDrive.setPower(turn);
        BackLeftDrive.setPower(turn);
        FrontRightDrive.setPower(-turn);
        BackRightDrive.setPower(-turn);
    }

    public void StopDriving(){
        FrontLeftDrive.setPower(0);
        BackLeftDrive.setPower(0);
        FrontRightDrive.setPower(0);
        BackRightDrive.setPower(0);
    }

    //WITH ENCODER DRIVE METHODS
    //THESE ARE SHITTT DONT USE THESE


    //                ༼ つ ◕_◕ ༽つ    ༼ つ ◕_◕ ༽つ     ༼ つ ◕_◕ ༽つ

    //                         move along nothing to see here
    public void encoderDriveForwards(double drivePower, int distance, boolean nom){

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

        drive(drivePower);

        while (FrontLeftDrive.isBusy() && BackLeftDrive.isBusy() && FrontRightDrive.isBusy() && BackRightDrive.isBusy()){
            // waiting until the target position is reached
            // if asked for nom, run nom while moving
            if(nom)nom();
        }
        // stop driving
        StopDriving();
        stopNom();

        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderDriveBackwards(double drivePower, int distance, boolean nom){
        encoderDriveForwards(-drivePower, -distance, nom);
    }

    public void encoderTurnLeft(double drivePower, int distance, boolean nom){
        // reset the encoders
        FrontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set to the target (distance)
        FrontLeftDrive.setTargetPosition(distance);
        BackLeftDrive.setTargetPosition(distance);
        FrontRightDrive.setTargetPosition(-distance);
        BackRightDrive.setTargetPosition(-distance);

        // set to RUN_TO_POSITION mode
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turn(drivePower);

        while (FrontLeftDrive.isBusy() && BackLeftDrive.isBusy() && FrontRightDrive.isBusy() && BackRightDrive.isBusy()){
            // waiting until the target position is reached
            // if asked for nom, run nom while moving
            if(nom)nom();
        }
        // stop driving
        StopDriving();
        stopNom();

        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderTurnRight(double drivePower, int distance, boolean nom){
        encoderTurnLeft(-drivePower, -distance, nom);
    }

    public void encoderStrafeRight(double drivePower, int distance, boolean nom){
        // reset the encoders
        FrontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set to the target (distance)
        FrontLeftDrive.setTargetPosition(distance);
        BackLeftDrive.setTargetPosition(-distance);
        FrontRightDrive.setTargetPosition(-distance);
        BackRightDrive.setTargetPosition(distance);

        // set to RUN_TO_POSITION mode
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        strafe(drivePower);

        while (FrontLeftDrive.isBusy() && BackLeftDrive.isBusy() && FrontRightDrive.isBusy() && BackRightDrive.isBusy()){
            // waiting until the target position is reached
            // if asked for nom, run nom while moving
            if(nom)nom();
        }
        // stop driving
        StopDriving();
        stopNom();

        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderStrafeLeft(double drivePower, int distance, boolean nom) {
        encoderStrafeRight(-drivePower, -distance, nom);
    }
    public RelicRecoveryVuMark getPicto() { //function to figure out which column it is
        OpenGLMatrix lastLocation = null;

        VuforiaLocalizer vuforia;
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "Aeba4Qn/////AAAAGahNOxzreUE8nItPWzsrOlF7uoyrR/qbjue3kUmhxZcfZMSd5MRyEY+3uEoVA+gpQGz5KyP3wEjBxSOAb4+FBYMZ+QblFU4byMG4+aiI+GeeBA+RatQXVzSduRBdniCW4qehTnwS204KTUMXg1ioPvUlbYQmqM5aPMx/2xnYN1b+htNBWV0Bc8Vkyspa0NNgz7PzF1gozlCIc9FgzbzNYoOMhqSG+jhKf47SZQxD6iEAtj5iAkWBvJwFDLr/EfDfPr3BIA6Cpb4xaDc0t4Iz5wJ/p4oLRiEJaLoE/noCcWFjLmPcw9ccwYXThtjC+7u0DsMX+r+1dMikBCZCWWkLzEyjWzy3pOOR3exNRYGZ0vzr";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; //look through back camera
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        relicTrackables.activate();

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        double vuStartTime = getRuntime();
        while (getRuntime() - vuStartTime < 3) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }
        telemetry.addData("column", vuMark);
        telemetry.update();
        return vuMark;
    }

}