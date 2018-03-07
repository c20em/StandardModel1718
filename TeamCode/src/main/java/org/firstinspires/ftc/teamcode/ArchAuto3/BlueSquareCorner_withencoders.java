package org.firstinspires.ftc.teamcode.ArchAuto3;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by student on 2/18/18.
 */



/**
 * Created by student on 2/16/18.
 */

@Autonomous(name="Blue Square Corner with Encoders", group="Autonomous")
@Disabled
public class BlueSquareCorner_withencoders extends BaseAutoFunctions {
    static double BOX_RIGHT_UP = .84;
    static double BOX_LEFT_UP = .1;
    static final double BOX_RIGHT_DOWN = .34;
    static final double BOX_LEFT_DOWN = .61;

    @Override
    public void runOpMode() throws InterruptedException {
        FrontLeftDrive = hardwareMap.get(DcMotor.class, "front_left");
        FrontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        BackLeftDrive = hardwareMap.get(DcMotor.class, "back_left");
        BackRightDrive = hardwareMap.get(DcMotor.class, "back_right");
        jewelServo = hardwareMap.get(Servo.class, "jewel_servo");
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

        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

  

        while(opModeIsActive()) {
            //jewel(true); /////TAKE THIS OUT WHEN USING THE JEWEL
            //delay(1000);   /////THIS IS JUST WHILE JEWEL SERVO IS UNDER REPAIR

//
//            //Knock off a jewel
//            TurnLeftwithEncoders(.2,40,false);
//            delay(30);
//            TurnRightwithEncoders(.2,40,false);
//            delay(300);
//
//            lift(.3, 200);
//            delay(200);
//            DriveBackwardwithEncoders(.5, 400, false);
//            delay(200);
//            TurnLeftwithEncoders(.3, 200, false);
//
//            flipOut();               //NECESSARY: need the delay in between for 1000ms to allow servo to actually get to position before being called back in
//            delay(600);
//            flipIn();
//            delay(600);
//            DriveBackwardwithEncoders(.3, 200, false);
//            delay(500);
//            DriveForwardwithEncoders(.3, 100, false);
//            delay(200);
//            StrafeLeftwithEncoders(.4, 40, false);
//            delay(500);
//            DriveBackwardwithEncoders(.3, 200, false);
//            delay(500);
//            DriveForwardwithEncoders(.5, 200, false);
//            break;

        }
    }
}
