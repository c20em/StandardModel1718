package org.firstinspires.ftc.teamcode.newomnomnomnomnom;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by student on 2/16/18.
 */

@Autonomous(name="Blue Square Corner", group="Autonomous")

public class BlueSquareCorner extends BaseAutoFunctions {
    private Servo rightBoxServo = null;
    private Servo leftBoxServo = null;
    private DcMotor lift = null;
    private DcMotor NomNomNom = null;


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



        waitForStart();

        jewelHit = true; /////TAKE THIS OUT WHEN USING THE JEWEL
                         /////THIS IS JUST WHILE JEWEL SERVO IS UNDER REPAIR

        while(opModeIsActive() && !jewelHit) {
            //jewel(true); /////TAKE THIS OUT WHEN USING THE JEWEL
            //sleep(1000);   /////THIS IS JUST WHILE JEWEL SERVO IS UNDER REPAIR
            leftBoxServo.setPosition(BOX_LEFT_UP+.1);      //Raising box and lift so we do not run into the end of the box
            rightBoxServo.setPosition(BOX_RIGHT_UP-.1);    //while getting off the balancing stone
            liftforTime(.8, 200);
            driveforTime(-.8, 1000);
            turnforTime(.6, 500);
            driveforTime(-.8, 100);
            autoFlip(true);
            liftforTime(-.8, 200);
            leftBoxServo.setPosition(BOX_LEFT_UP);
            rightBoxServo.setPosition(BOX_RIGHT_UP);
            driveforTime(.8, 750);
            driveBackNomming(.4);
            driveforTime(-.6, 400);
            strafeforTime(1, 300);
            driveforTime(-.2, 200);
            autoFlip(true);
            leftBoxServo.setPosition(BOX_LEFT_UP);
            rightBoxServo.setPosition(BOX_RIGHT_UP);


        }
    }
}
