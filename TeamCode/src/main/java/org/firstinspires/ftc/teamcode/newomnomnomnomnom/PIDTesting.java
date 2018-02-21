package org.firstinspires.ftc.teamcode.newomnomnomnomnom;

/**
 * Created by student on 2/19/18.
 */

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.util.Range;

        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name = "Gyro Tester", group = "Sensor")
public class PIDTesting extends BaseAutoFunctions {
    @Override
    public void runOpMode() throws InterruptedException {
        declare();
        initGyro();
        waitForStart();

        while (opModeIsActive()) {
            turn(90);
            delay(2000);
        }
    }

    public void turn90(boolean right) throws InterruptedException {
        double P = -0.2, I = 0, D = 0;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        float current = to360(angles.firstAngle);
        float goal = to360(right ? current - 90 : current + 90);
        double now = getRuntime();
        double timeChange;
        double integral = 0.0;
        double prevError = 0.0;

        while(goal != current) {
            double error = goal - current;

            double change = error - prevError;
            prevError = error;

            timeChange = timeChange(now);
            now = getRuntime();

            integral += error * timeChange;

            double PID = P*error + I*integral + D*change;

            turn(Range.clip(PID, -1, 1));
            current = to360(angles.firstAngle);
        }
    }

    public float to360(float angle) {
        if (angle < 0) {
            return 360 + angle;
        } else if (angle > 360) {
            return angle - 360;
        }
        return angle;
    }

    public double timeChange(double lastTime) {
        return (getRuntime() * 1000) - lastTime;
    }

}