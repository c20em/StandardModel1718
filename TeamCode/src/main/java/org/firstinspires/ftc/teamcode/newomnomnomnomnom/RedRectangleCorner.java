package org.firstinspires.ftc.teamcode.newomnomnomnomnom;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by student on 2/16/18.
 */

@Autonomous(name="Red Rectangle Corner", group="Autonomous")

public class RedRectangleCorner extends BaseAutoFunctions {
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





        waitForStart();

        jewelHit = false; /////TAKE THIS OUT WHEN USING THE JEWEL
        /////THIS IS JUST WHILE JEWEL SERVO IS UNDER REPAIR

        while(opModeIsActive()) {
            telemetry.addLine("made it this far");
            telemetry.update();
            liftIn.setPosition(.9);             //Relic Blocker
            elbowServo.setPosition(.2);         //Relic arm up
            sleep(1000);                         //breif pause so that wall servo does not interfere with relic arm release
            wallServo.setPosition(.3);          //Wall servo out

            //jewel(true); /////TAKE THIS OUT WHEN USING THE JEWEL
            //sleep(1000);   /////THIS IS JUST WHILE JEWEL SERVO IS UNDER REPAIR
//////////
            driveforTime(-.8, 300);
            sleep(500);
            strafeforTime(-.95, 650);
            sleep(500);
            driveforTime(-.4, 1000);
            sleep(500);
            driveforTime(.4, 400);
            sleep(500);
            flipIn();               //NECESSARY: need the sleep in between for 1000ms to allow servo to actually get to position before being called back in
            sleep(500);
            flipOut();
            sleep(500);
            strafeforTime(-.95, 200);
            driveforTime(-.3, 600);


            //FIRST GLYPH PLACED
///////////
            strafeforTime(-1, 500);
            nomOn(nomPower, 50);
            sleep(200);

            driveforTime(-.2, 1500);//align with wall
            driveforTime(-.6, 800);//align with wall

            sleep(1000);
            driveforTime(.6,500);
            strafeforTime(1,200);
            //second glyph?
            driveforTime(.3, 1600);
            nomOn(nomPower,500);
            drivewithNom(-.2, 2000);
            nomOn(1, 400);
            //third glyph?
            driveforTime(.3, 1600);
            nomOn(nomPower,500);
            drivewithNom(-.2, 2000);
            nomOn(1, 400);


            strafeforTime(1, 800);
            sleep(500);
            driveforTime(-.4, 1400);
            sleep(500);
            driveforTime(.4, 400);
            sleep(500);
            flipIn();               //NECESSARY: need the sleep in between for 1000ms to allow servo to actually get to position before being called back in
            sleep(1000);
            flipOut();
            sleep(1000);
            strafeforTime(-.95, 200);
            driveforTime(-.3, 600);
            sleep(100);
            driveforTime(.9, 100);
            sleep(100);
            driveforTime(-.3, 600);
            sleep(100);
            driveforTime(.9, 100);


            break;
//

        }
    }
}
