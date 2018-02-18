package org.firstinspires.ftc.teamcode.newomnomnomnomnom;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by student on 2/16/18.
 */

@Autonomous(name="Red Square Corner", group="Autonomous")
public class RedSquareCorner extends BaseAutoFunctions {
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

            //jewel(true); /////TAKE THIS OUT WHEN USING THE JEWEL
            //sleep(1000);   /////THIS IS JUST WHILE JEWEL SERVO IS UNDER REPAIR

            liftforTime(.5, 400);
            sleep(200);
            driveforTime(-.4, 500);
            sleep(200);
            liftforTime(-.5, 400);
            sleep(500);
            turnforTime(-.5, 650);
            sleep(500);
            driveforTime(-.4, 800);
            sleep(500);
            driveforTime(.4, 400);

            flipIn();               //NECESSARY: need the sleep in between for 1000ms to allow servo to actually get to position before being called back in
            sleep(1000);
            flipOut();
            sleep(1000);

            driveforTime(-.3, 1000);
            sleep(500);
            strafeforTime(-1, 300);
            sleep(500);
            driveforTime(-.3, 1000);
            sleep(500);
            telemetry.addLine("going for 2");
            telemetry.update();
            //BEGIN 2nd GLYPH!!


            liftIn.setPosition(.9);             //Relic Blocker
            elbowServo.setPosition(.2);         //Relic arm up
            sleep(1000);                         //breif pause so that wall servo does not interfere with relic arm release
            wallServo.setPosition(.3);          //Wall servo out

            sleep(500);
            drivewithNom(.7,400);
            sleep(100);
            driveforTime(-.7, 400);
            sleep(500);
            drivewithNom(.6, 1500);
            telemetry.addLine("drive forward with nom");
            telemetry.update();
            nomOn(nomPower,400);

            drivewithNom(-.4, 1000);

            nomOn(nomPower, 600);

            drivewithNom(.6, 400);

            nomOn(nomPower,400);

            drivewithNom(-.4, 800);

            sleep(500);
            strafeforTime(.75, 300);

            driveforTime(.4, 400);

            flipIn();               //NECESSARY: need the sleep in between for 1000ms to allow servo to actually get to position before being called back in
            sleep(1000);
            flipOut();
            sleep(1000);
            driveforTime(.3, 1000);
            sleep(500);
            strafeforTime(-1, 800);
            sleep(500);
            driveforTime(-.3, 1000);
            sleep(500);
            break;
//

        }
    }
}
