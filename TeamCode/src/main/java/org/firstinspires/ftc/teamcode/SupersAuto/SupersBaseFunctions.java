package org.firstinspires.ftc.teamcode.SupersAuto;

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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

abstract class SupersBaseFunctions extends LinearOpMode {

    //***********************************HARDWARE INSTANTIATIONS************************************

    //motor + servos
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

    //imu
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    //servo final double positions
    static final double JEWEL_DOWN_POS = 0.40;
    static final double JEWEL_MID_POS = .9;
    static final double JEWEL_UP_POS = 1;
    static final double SIDE_JEWEL_NEUTRAL_POS = 0.6;
    static double BOX_RIGHT_DOWN = .84;
    static double BOX_LEFT_DOWN = .1;
    static final double BOX_RIGHT_UP = .34;
    static final double BOX_LEFT_UP = .61;
    static final double ELBOW_UP = .1;
    static final double JEWEL_TURNCW_POS = .80;
    static final double JEWEL_TURNMID_POS = .63;
    static final double JEWEL_TURNCCW_POS = .38;

    //vuforia
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    VuforiaLocalizer.Parameters vufParameters = null;
    VuforiaTrackables relicTrackables = null;
    VuforiaTrackable relicTemplate = null;

    //other things
    double nomPower = -0.95;
    boolean canSeeJewel = false;
    ElapsedTime clock = new ElapsedTime();
    public double veryStartAngle;
    static final double COLUMN_TURN_ANGLE = 15;

    //*************************************INITIALIZATION FUNCTIONS*********************************

    public void declare() {
        //motor and servo declarations
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

        //motor direction behavior
        FrontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        BackLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        BackRightDrive.setDirection(DcMotor.Direction.FORWARD);
        FrontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        NomNomNom.setDirection(DcMotor.Direction.REVERSE);

        //motor zero power behavior
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
        vufParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        vufParameters.vuforiaLicenseKey = "Aeba4Qn/////AAAAGahNOxzreUE8nItPWzsrOlF7uoyrR/qbjue3kUmhxZcfZMSd5MRyEY+3uEoVA+gpQGz5KyP3wEjBxSOAb4+FBYMZ+QblFU4byMG4+aiI+GeeBA+RatQXVzSduRBdniCW4qehTnwS204KTUMXg1ioPvUlbYQmqM5aPMx/2xnYN1b+htNBWV0Bc8Vkyspa0NNgz7PzF1gozlCIc9FgzbzNYoOMhqSG+jhKf47SZQxD6iEAtj5iAkWBvJwFDLr/EfDfPr3BIA6Cpb4xaDc0t4Iz5wJ/p4oLRiEJaLoE/noCcWFjLmPcw9ccwYXThtjC+7u0DsMX+r+1dMikBCZCWWkLzEyjWzy3pOOR3exNRYGZ0vzr";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        vufParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(vufParameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        relicTrackables.activate();
        telemetry.addData("relic", "activated");
    }

    //***************************************MOTION FUNCTIONS***************************************

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

    public void driveforTime(double power, int time){
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

    public void nom (int nomDirection) {
        NomNomNom.setPower(nomPower * nomDirection);
        pushBackServoLeft.setPower(-1);
        pushBackServoRight.setPower(1);
    }

    public void stopNom() {
        NomNomNom.setPower(0);
        pushBackServoLeft.setPower(0);
        pushBackServoRight.setPower(0);
    }

    public void nomDrive(double power) {
        nom(1);
        drive(power);
    }

    public void nomDriveForTime(double power, int time){
        nomDrive(power);
        sleep(time);
        StopDriving();
        stopNom();
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

    //**********************************VUFORIA FUNCTIONS*******************************************

    public RelicRecoveryVuMark getPicto() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("Mark",vuMark);
            telemetry.update();
            return vuMark;
        }
        clock.reset();
        while (clock.milliseconds() < 2000) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("Mark",vuMark);
                telemetry.update();
                return vuMark;
            }
        }
        telemetry.addData("Mark",vuMark);
        telemetry.update();
        return vuMark;
    }

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

    //******************************GYRO + TURNING FUNCTIONS****************************************

    public void strafeforTimeGyro(double power, int time) throws InterruptedException {
        double startingAngle = currentAngle();
        double startingTime = getRuntime();
        while(opModeIsActive()&&((getRuntime()-startingTime)*1000<time)){
            strafeGyro(power,currentAngle()-startingAngle);
        }
        StopDriving();
    }

    public void strafeGyro(double power, double error) {
        double errorMultiplier = 1;
        FrontLeftDrive.setPower(power + error*errorMultiplier);
        BackLeftDrive.setPower(-power + error*errorMultiplier);
        FrontRightDrive.setPower(-power + error*errorMultiplier);
        BackRightDrive.setPower(power + error*errorMultiplier);
    }

    double currentAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }

    public void turnAngle(double rawAngle){
        double angle = rawAngle % 360;
        if(angle == 0) return;
        else if(angle > 0 && angle < 180) turnAngleCW(angle);
        else if (angle < -180) turnAngleCW(angle+360);
        else if(angle < 0 && angle > -180) turnAngleCCW(-angle);
        else turnAngleCCW(360-angle);
    }

    public void turnAngleCW(double angle) {
        double startingAngle = currentAngle();
        while ((getAngleDiff(startingAngle, currentAngle()) < angle -4)&&opModeIsActive()) {
            double difference = ((angle - getAngleDiff(startingAngle, currentAngle())) / (180));
            telemetry.addData("difference", difference);
            telemetry.update();
            if (difference > .15) turn(difference);
            else turn(.22);
            telemetry.addData("I BROKE",opModeIsActive());
            telemetry.update();
        }
        StopDriving();
    }

    public void turnAngleCCW(double angle) {
        while(opModeIsActive()) {
            double startingAngle = currentAngle();
            while (getAngleDiff(startingAngle, currentAngle()) < angle -4) {
                double difference = ((angle - getAngleDiff(startingAngle, currentAngle())) / (180));
                telemetry.addData("difference", difference);
                telemetry.update();
                if (difference > .15) turn(-difference);
                else turn(-.22);
                if(!opModeIsActive()){
                    telemetry.addData("I BROKE",opModeIsActive());
                    telemetry.update();
                    break;
                }
            }
            StopDriving();
            break;
        }
    }

    public double getAngleDiff(double angle1, double angle2) {
        if(Math.abs(angle1 - angle2) < 180.0)
            return Math.abs(angle1-angle2);
        else if(angle1 > angle2)
        {
            angle1 -= 360;
            return Math.abs(angle2-angle1);
        }
        else
        {
            angle2 -= 360;
            return Math.abs(angle1-angle2);
        }
    }

    public void setStartAngle(){
        veryStartAngle = currentAngle();
    }

    public void faceCryptoRectangle(){
        turn(currentAngle()-veryStartAngle);
    }

    //*******************************SEQUENCE MOTION FUNCTIONS******************************************
    public void jewelSequence(boolean teamBlue) throws InterruptedException {
        boolean jewelBlue;
        jewelSideServo.setPosition(JEWEL_TURNMID_POS);
        sleep(100);
        jewelServo.setPosition(JEWEL_MID_POS);
        sleep(500);
        jewelServo.setPosition(JEWEL_DOWN_POS);
        //read color
        int red = 0;
        int blue = 0;
        for (int i = 0; i < 50; i++) {
            if (colorSensor.red()  > colorSensor.blue() && colorSensor.red() >.1) red++;
            if (colorSensor.red() < colorSensor.blue()&& colorSensor.blue() >.1) blue++;
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
            if(teamBlue) jewelSideServo.setPosition(jewelSideServo.getPosition()-.12);
            else if(!teamBlue) jewelSideServo.setPosition(jewelSideServo.getPosition()+.12);
        } else if (!jewelBlue){
            if(teamBlue) jewelSideServo.setPosition(jewelSideServo.getPosition()+.12);
            else if(!teamBlue) jewelSideServo.setPosition(jewelSideServo.getPosition()-.12);
        }
        sleep(200);

        //turn back
        jewelSideServo.setPosition(JEWEL_TURNMID_POS);
        sleep(100);
        jewelServo.setPosition(JEWEL_UP_POS);
        sleep(600);
    }

    public void placeGlyphSequence(){
        driveforTime(.3,200);
        flipOut();
        sleep(800);
        driveforTime(-.3,700); //200 to negate the above and then drives 500
        sleep(200);
        driveforTime(.3, 500);
        sleep(200);
        driveforTime(-.3, 600); //pushes in final time
        sleep(200);
        driveforTime(.3, 500); //leaves
        sleep(200);
        flipIn();
        sleep(100);
//        sleep(300);
//        driveforTime(-.3, 800);
    }

    public void placeSpaciousGlyphSequence(){
        driveforTime(.6,200);
        flipOut();
        sleep(800);
        driveforTime(-.3,700); //200 to negate the above and then drives 500
        sleep(200);
        driveforTime(.3, 500);
        sleep(200);
        driveforTime(-.5, 600); //pushes in final time
        sleep(200);
        driveforTime(.5, 500); //leaves
        sleep(200);
        flipIn();
        sleep(100);
    }

    public void servoStartSequence(){
        liftIn.setPosition(.9);             //Relic Blocker//Relic arm up
    }

    public void turnToColumnSequence(RelicRecoveryVuMark column, int turn) throws InterruptedException {
        turnAngle(currentAngle() - veryStartAngle);
        turnAngle(turn);
        //TURN TO THE CORRECT COLUMN
        if (column == RelicRecoveryVuMark.CENTER || column == RelicRecoveryVuMark.UNKNOWN) {
            strafeforTime(-.9, 400);
            turnAngle(-COLUMN_TURN_ANGLE);
        } else if (column == RelicRecoveryVuMark.LEFT) {
            turnAngle(-COLUMN_TURN_ANGLE);//fill w left value
        } else if (column == RelicRecoveryVuMark.RIGHT) {
            turnAngle(COLUMN_TURN_ANGLE);//fill w right value
        }

        sleep(200);
        driveforTime(-.5,400);
        sleep(300);
    }

    public void turnToSecondColumnSequence(RelicRecoveryVuMark column, boolean square) throws InterruptedException {
        nomDriveForTime(-.3, 700); //?
        sleep(200); //?
        nomDriveForTime(.3, 600);  //?
        if(square){
            turnAngle(currentAngle() - (veryStartAngle-90));
            strafeforTime(.8,100);
        }
        else turn(currentAngle()-veryStartAngle);

        //TURN TO THE CORRECT COLUMN
        if (column == RelicRecoveryVuMark.LEFT || column == RelicRecoveryVuMark.CENTER || column == RelicRecoveryVuMark.UNKNOWN) {
            turnAngle(COLUMN_TURN_ANGLE+6);//fill w left value
        } else if (column == RelicRecoveryVuMark.RIGHT) {
            turnAngle(-COLUMN_TURN_ANGLE);//fill w right value
        }

        sleep(200);
        driveforTime(-.5,400);
        sleep(300);
    }

    public void returntoCenterSequence(RelicRecoveryVuMark column, boolean isSquare) throws InterruptedException {
        sleep(300);

        if (column == RelicRecoveryVuMark.CENTER || column == RelicRecoveryVuMark.UNKNOWN) {
            turnAngle(COLUMN_TURN_ANGLE);
            sleep(100);
            strafeforTime(.9, 400);
        } else if (column == RelicRecoveryVuMark.LEFT) {
            turnAngle(COLUMN_TURN_ANGLE);//fill w left value
        } else if (column == RelicRecoveryVuMark.RIGHT) {
            turnAngle(-COLUMN_TURN_ANGLE);//fill w right value
        }
        if(isSquare) {
            turnAngle(currentAngle() - (veryStartAngle - 90));
        }
        else turnAngle(currentAngle()-veryStartAngle);

    }

    public void getNewGlyphSquareSequence(){
//        NomNomNom.setPower(-1);
//        sleep(400);
        nom(1);
        sleep(200);
        nom(-1);
        sleep(200);
        nomDriveForTime(.7, 1000);
        sleep(100);
        nomDriveForTime(-.3, 300);
        sleep(100);
        nomDriveForTime(.3, 300);
        sleep(100);
        nomDriveForTime(-.7, 1000);
        sleep(100);
        nom(1);
        sleep(300);
    }

    public void getNewGlyphRectangleSequence(int direction) throws InterruptedException {
        nom(-1);
        sleep(100); //delays
        nom(1);
        strafeforTime(direction*.8, 400); //strafes to set up
        sleep(100);
        turnAngle(currentAngle()-veryStartAngle); //resets angle every time
        sleep(100);
        driveforTime(.6, 1200); //goes forward first time
        sleep(100);
        turnAngle(currentAngle()-(veryStartAngle-(45*direction))); //turns 45 degrees
        sleep(100);
        driveforTime(.3, 300); //goes forward to get blocks
        sleep(300);
        driveforTime(-.3, 300); //goes backwards
        sleep(100);
        turnAngle(currentAngle()-(veryStartAngle-(45*direction))); //resets the 45 degree angle
        sleep(100);
        driveforTime(.6, 300); //goes forward
        sleep(300);
        driveforTime(-.6, 300); //backs up
        sleep(100);
        turnAngle(currentAngle()-veryStartAngle); //resets to straight (aligned w crypto)
        sleep(100);
        driveforTime(-.6, 1200); //goes backward
//        sleep(100);
//        turnAngle(currentAngle()-veryStartAngle);
//        sleep(100);
//        nomDriveForTime(.3, 2800); //goes forward to try again
//        sleep(100);
//        turnAngle(currentAngle()-veryStartAngle);
//        sleep(100);
//        nomDriveForTime(-.3, 1900); //drives back a final time
        sleep(100);
        turnAngle(currentAngle()-veryStartAngle); //resets to straight
        sleep(100);
        strafeforTime(direction*-.8, 600); //strafes to put in cryptobox
        nom(0);
    }

    public void turnToColumnAbbySequence(RelicRecoveryVuMark column, int turn) throws InterruptedException {
        //IT NEEDS TO STRAFE LESS
        //ON THE FIRST GLYPH IT PACED IT TOO SHORT
        turnAngle(currentAngle() - veryStartAngle);
        turnAngle(turn);
        //TURN TO THE CORRECT COLUMN
        if (column == RelicRecoveryVuMark.CENTER || column == RelicRecoveryVuMark.UNKNOWN) {
            strafeforTime(-.9, 350);
            turnAngle(-COLUMN_TURN_ANGLE-6);
        } else if (column == RelicRecoveryVuMark.LEFT) {
            turnAngle(-COLUMN_TURN_ANGLE);//fill w left value
        } else if (column == RelicRecoveryVuMark.RIGHT) {
            turnAngle(COLUMN_TURN_ANGLE);//fill w right value
        }

        sleep(200);
        driveforTime(-.5,400);
        sleep(300);
    }

    public void turnToSecondColumnAbbySequence(RelicRecoveryVuMark column, boolean square, boolean isBlue) throws InterruptedException {
        //ON THE SECOND GLYPH IT WAS TOO CLOSE TO THE BALANCING STONE!!! 3/16/18 ON 9:13PM

        nomDriveForTime(-.3, 700);
        sleep(200);
        nomDriveForTime(.3, 600);
        if(square){
            if(!isBlue)turnAngle(currentAngle() - (veryStartAngle-90));
            if(isBlue)turnAngle(currentAngle() - (veryStartAngle+90));
            strafeforTime(.8,100);
        }
        else turn(currentAngle()-veryStartAngle);

        //TURN TO THE CORRECT COLUMN
        if (column == RelicRecoveryVuMark.LEFT) {
            strafeforTime(-.8,500); // WE WANT THIS TO STRAFE LESS
            turnAngle(COLUMN_TURN_ANGLE);//fill w left value
        } else if (column == RelicRecoveryVuMark.CENTER || column == RelicRecoveryVuMark.UNKNOWN) {
            turnAngle(COLUMN_TURN_ANGLE);//fill w center value
        } else if (column == RelicRecoveryVuMark.RIGHT) {
            strafeforTime(.6,100);
            turnAngle(-COLUMN_TURN_ANGLE);//fill w right value
        }

        sleep(200);
        driveforTime(-.5,400);
        sleep(300);
    }

}