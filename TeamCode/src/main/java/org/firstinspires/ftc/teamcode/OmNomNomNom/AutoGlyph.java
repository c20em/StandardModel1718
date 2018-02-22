package org.firstinspires.ftc.teamcode.OmNomNomNom;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Skipper.BlueLeftAuto;

/**
 * Created by student on 1/14/18.
 */
@Autonomous(name="Basic Glyph", group="Autonomous")
@Disabled
public class AutoGlyph extends LinearOpMode {

    //Vuforia
    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FrontLeftDrive = null;
    private DcMotor FrontRightDrive = null;
    private DcMotor BackLeftDrive = null;
    private DcMotor BackRightDrive = null;
    private DcMotor lift = null;
    private DcMotor NomNomNom = null;
    private Servo rightBoxServo = null;
    private Servo leftBoxServo = null;

    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;

    @Override public void runOpMode() {

        //Gyro
        BNO055IMU.Parameters gyroParams = new BNO055IMU.Parameters();
        gyroParams.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        gyroParams.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyroParams.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        gyroParams.loggingEnabled      = true;
        gyroParams.loggingTag          = "IMU";
        gyroParams.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(gyroParams);

        //Vuforia
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "ARIQmOP/////AAAAGeaba2DIeEOEva4DaeuBipo8BDjTCmCmGklmJpugOHU/lBLfwAwKFTHfyR4+kMn92unZ9GnhqsSfZ29S+2bhnjHqOQ9vGtiOH6h1E91Ag7K0Rvn+2di7v89KBfARxsadYZoBfFAmvavXt1PnpOtuSctHl8VrPpocTmZmOI/Om7iuvofHQNz3zorwxMAS69WHAIr5YJN1XHJ+8cgXT5RoBECCIFrGWpLhjyvDcU4JKJH8SUxZiw5lXYjlH7pNIHPvetpHGKQ1Ku8kXc0ttJ4Kpbu5Zsxi9uuZiA/f+RazLEtHcTGd0Hhi19PA4XFBjlWONV/rSr/1YJNHgfNCMSW/MNILB0MGD0V4qc5lNCeD4KRY\n";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);


        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        //Motors/servos
        FrontLeftDrive = hardwareMap.get(DcMotor.class, "front_left");
        FrontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        BackLeftDrive = hardwareMap.get(DcMotor.class, "back_left");
        BackRightDrive = hardwareMap.get(DcMotor.class, "back_right");
        lift = hardwareMap.get(DcMotor.class, "lift");
        NomNomNom = hardwareMap.get(DcMotor.class, "nom");
        rightBoxServo = hardwareMap.get(Servo.class, "right_box_servo");
        leftBoxServo = hardwareMap.get(Servo.class, "left_box_servo");

        //Set motors so they naturally run 'forward'
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

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        FrontLeftDrive.setPower(0);
        BackLeftDrive.setPower(0);
        BackRightDrive.setPower(0);
        FrontRightDrive.setPower(0);

        waitForStart();

        relicTrackables.activate();

        while(opModeIsActive()) {

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

            while (vuMark == RelicRecoveryVuMark.UNKNOWN) {
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                    telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                } else {
                    telemetry.addData("VuMark", "not visible");
                }

                telemetry.update();
            }

            forward(.5, 1000);

            stopDrive();

        }

        telemetry.addData("Running", "False");
        telemetry.update();
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    void forward(double speed, int time) {

        FrontLeftDrive.setPower(speed);
        BackLeftDrive.setPower(speed);
        BackRightDrive.setPower(speed);
        FrontRightDrive.setPower(speed);

        sleep(time);

    }

    void stopDrive() {

        FrontLeftDrive.setPower(0);
        BackLeftDrive.setPower(0);
        BackRightDrive.setPower(0);
        FrontRightDrive.setPower(0);

    }

    public void spinClockwise(double power) {
        FrontLeftDrive.setPower(power);
        BackLeftDrive.setPower(power);
        BackRightDrive.setPower(-power);
        FrontRightDrive.setPower(-power);
    }

    public void spinCounter(double power) {
        FrontLeftDrive.setPower(-power);
        BackLeftDrive.setPower(-power);
        BackRightDrive.setPower(power);
        FrontRightDrive.setPower(power);
    }

    public void turnToAngle(float angle) {
        float current;

        angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        current = angles.firstAngle;

        if (angle > current) {
            while (angle >= current + 5 && angle <= current -5) {
                spinClockwise(.75);
                current = angles.firstAngle;
            }
        } else {
            while (angle >= current + 5 && angle <= current -5) {
                spinCounter(.75);
                current = angles.firstAngle;
            }
        }
    }

}
