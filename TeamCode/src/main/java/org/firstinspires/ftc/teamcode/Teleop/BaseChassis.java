package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    private DcMotor relicArm = null;
    private Servo rightBoxServo = null;
    private Servo leftBoxServo = null;
    private CRServo pushBackServoLeft = null;
    private CRServo pushBackServoRight = null;
    private Servo wallServo = null;
    private Servo liftIn = null;
    private Servo elbowServo = null;
    private Servo handServo = null;
    public Servo jewelServo = null;






    //static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
//    static final int    CYCLE_MS    =   75;
    static final double NOM_FORWARD_POWER = 1;
    static final double NOM_BACKWARD_POWER = -1;
    static final double BOX_RIGHT_DOWN = .34;
    static final double BOX_LEFT_DOWN = .61;
    static final double BOX_RIGHT_UP = .84;
    static final double BOX_LEFT_UP = .1;
    static final double Push_Back_Power = 1;
    static final double ELBOW_UP = .1;
    static final double ELBOW_DOWN = .58;
    static final double JEWEL_UP_POS = 0.6;




    // Define class members
    double strafepower = 1;
    static final double NOM_POWER = 1;
    boolean wallout = false;
    boolean relicGang = false;

    controllerPos previousDrive = controllerPos.ZERO;

    //boolean box position

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
        relicArm = hardwareMap.get(DcMotor.class, "relic_arm");
        rightBoxServo = hardwareMap.get(Servo.class, "right_box_servo");
        leftBoxServo = hardwareMap.get(Servo.class, "left_box_servo");
        pushBackServoRight = hardwareMap.get(CRServo.class, "push_back_servo_right");
        pushBackServoLeft = hardwareMap.get(CRServo.class, "push_back_servo_left");
        wallServo = hardwareMap.get(Servo.class, "wall_servo");
        liftIn = hardwareMap.get(Servo.class, "lift_in");
        elbowServo = hardwareMap.get(Servo.class, "elbow_servo");
        handServo = hardwareMap.get(Servo.class, "hand_servo");
        jewelServo = hardwareMap.get(Servo.class, "jewel_servo");



        FrontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        BackLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        BackRightDrive.setDirection(DcMotor.Direction.FORWARD);
        FrontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);
        NomNomNom.setDirection(DcMotor.Direction.FORWARD);
        relicArm.setDirection(DcMotor.Direction.FORWARD);
        FrontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        NomNomNom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relicArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("right up", leftBoxServo.getPosition());
        telemetry.addData("left up", rightBoxServo.getPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        liftIn.setPosition(.9);             //Relic Blocker
        elbowServo.setPosition(ELBOW_UP);         //Relic arm up
        sleep(1000);                         //breif pause so that wall servo does not interfere with relic arm release
        wallServo.setPosition(.78);          //Wall servo out
        liftIn.setPosition(.9);             //Relic Blocker

        while (opModeIsActive()) {
            jewelServo.setPosition(JEWEL_UP_POS);

            telemetry.addData("x stick", gamepad1.left_stick_x);
            telemetry.addData("y stick", gamepad1.left_stick_y);
            if (relicGang){
                telemetry.addLine("Relic Engaged");
                relic();
                if(gamepad1.right_bumper) {
                    relicGang = false;
                }
            }else {
                telemetry.addLine("Relic Disengaged");
                flip();
                moveLift();
                if (gamepad1.left_bumper) {
                    relicGang = true;
                    wallServo.setPosition(.3);
                    liftIn.setPosition(.3);
                }
            }
            telemetry.addData("elbow servo:", elbowServo.getPosition());

            nom();
            wall();
            moveRobot();
            pushBack();
            telemetry.update();
            idle();
        }
    }

    public enum controllerPos {
        STRAFE_RIGHT, STRAFE_LEFT, DRIVE_FOWARD, DRIVE_BACK, TURN_RIGHT, TURN_LEFT, ZERO;
    }


    //DRIVER CONTROL
                                         //MOTORS


    public void moveRobot() {
        double drive = -gamepad1.left_stick_y;
    double turn = -gamepad1.right_stick_x/1.2;

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

     public void moveLift() {
         double LiftPower;
         if(gamepad2.right_bumper) {                    //GAMEPAD2
             LiftPower = 1;
         } else if (gamepad2.left_bumper) {             //GAMEPAD2
             LiftPower = -1;
         } else {
             LiftPower = 0;
         }
         lift.setPower(LiftPower);
     }

     public void nom() {
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
               telemetry.addLine("no nom :(");
         }

         if(gamepad1.right_stick_button) {                 //Backup teleop control in case we do not run auto for some reason
             wallout = true;              //when wallout=true the relic arm servo, the wall servo, and the liftIn servo will
         }                                //all go to their starting positions
     }
    public void relic() {
        if (Math.abs(gamepad2.right_stick_y) > .2) {
            relicArm.setPower(Range.clip(gamepad2.right_stick_y, -1, 1));
        }else{
            relicArm.setPower(0);
        }
        if (gamepad2.y) {
            elbowServo.setPosition(ELBOW_UP);
        } else if (gamepad2.a) {
            elbowServo.setPosition(ELBOW_DOWN);
        } else if (gamepad2.x) {
            handServo.setPosition(.8);
        } else if (gamepad2.b) {
            handServo.setPosition(.3);
        } else if(gamepad2.right_bumper) {
            elbowServo.setPosition(elbowServo.getPosition() - .001);
        } else if(gamepad2.left_bumper) {
            elbowServo.setPosition(elbowServo.getPosition() + .001);
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


                                        //SERVOS


    public void flip() {
        telemetry.addLine("running flip()");
        telemetry.addLine("Box Servo Left: " + leftBoxServo.getPosition());
        telemetry.addLine("Box Servo Right: " + rightBoxServo.getPosition());
        if (gamepad2.y) {                                         //GAMEPAD2
            leftBoxServo.setPosition(BOX_LEFT_DOWN);
            rightBoxServo.setPosition(BOX_RIGHT_DOWN);
        } else if (gamepad2.a) {                                  //GAMEPAD2
            leftBoxServo.setPosition(BOX_LEFT_UP);
            rightBoxServo.setPosition(BOX_RIGHT_UP);
        }
        if (gamepad2.b && leftBoxServo.getPosition() > BOX_LEFT_UP && rightBoxServo.getPosition() < BOX_RIGHT_UP) {
            leftBoxServo.setPosition(leftBoxServo.getPosition() - .01);//GAMEPAD2
            rightBoxServo.setPosition(rightBoxServo.getPosition() + .01);
//            } else if (gamepad2.x && leftBoxServo.getPosition() < BOX_LEFT_DOWN && rightBoxServo.getPosition() > BOX_RIGHT_DOWN) {
//                leftBoxServo.setPosition(leftBoxServo.getPosition() + .01);//GAMEPAD2
//                rightBoxServo.setPosition(rightBoxServo.getPosition() - .01);
        } else if (gamepad2.x) {
            leftBoxServo.setPosition(BOX_LEFT_UP + .1);//GAMEPAD2
            rightBoxServo.setPosition(BOX_RIGHT_UP - .1);
        }
    }

    public void wall() {
        if (wallout) {                                   //OPENING EXPANSE SEQUENCE
            liftIn.setPosition(.9);             //Relic Blocker
//            elbowServo.setPosition(ELBOW_UP);        //Relic arm up
            sleep(1000);                         //breif pause so that wall servo does not interfere with relic arm release
            wallServo.setPosition(.78);          //Wall servo out
            wallout = false;
        }
    }
    public void pushBack(){
        if(gamepad2.dpad_up){                               //GAMEPAD2
            pushBackServoLeft.setPower(Push_Back_Power);
            pushBackServoRight.setPower(-Push_Back_Power);
        }
        else if(gamepad2.dpad_down){                         //GAMEPAD2
            pushBackServoLeft.setPower(-Push_Back_Power);
            pushBackServoRight.setPower(Push_Back_Power);
        }
        else{
            pushBackServoLeft.setPower(0);
            pushBackServoRight.setPower(0);
        }
    }


    public enum boxPosition {
        HALF_UP, UP, DOWN;
    }
}




