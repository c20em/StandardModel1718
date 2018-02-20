package org.firstinspires.ftc.teamcode.newomnomnomnomnom;

/**
 * Created by student on 2/19/18.
 */

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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

        }
    }

    public void turn90(boolean right) {
        int P, I, D = 1;
        int integral, previous_error, setpoint = 0;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        float current = to360(angles.firstAngle);
        float goal = to360(right ? current - 90 : current + 90);
        double now = getRuntime();
        double timeChange;

        while () {
            double error = goal - current;
            timeChange = timeChange(now);
            now = getRuntime();
            integral += error * timeChange;
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

    public void move(double pow, double rot) {
        rot = rot.clipRange(-1, 1);

        double[] V = {rot, -rot, rot, -rot}; // contains motor powers for each motor.

    /*
     * because of adding/subtracting rotation, these numbers could be between [-2,2].
     * To get around this, find the maximum motor power, and divide all of them by that
     * so that the proportions stay the same but now it's between [-1,1].
     */

        // find max
        double m = 0.0;
        for (double v : V)
            if (Math.abs(v) > m)
                m = v;

        double mult = Math.abs(rot); // If we're just rotating, pow will be 0
        // adjust values, still keeping power in mind.
        if (m != 0) // if the max power isn't 0 (can't divide by 0)
            for (int i = 0; i < V.length; i++)
                V[i] = Math.abs(mult) * (V[i] / Math.abs(m));

        // finally, set motor powers.
        FL.setPower(FtcUtil.motorScale(V[0]));
        FR.setPower(FtcUtil.motorScale(V[1]));
        BL.setPower(FtcUtil.motorScale(V[2]));
        BR.setPower(FtcUtil.motorScale(V[3]));

    }

    double integral = 0.0; // Accumulation of error over time. Used in PID controller.
    double lastError = 0.0;

    /**
     * Use PID in order to move directly in the desired angle, without rotation.
     *
     * @param pow    Motor power (approximate speed)
     * @param angle  Desired angle
     * @param actual Orientation/rotation measurement
     * @param target Desired orientation/rotation
     */
    public void moveStraight(double pow, double angle, double actual, double target) {
        double error = actual - target;
        integral += error;

        double change = (error - lastError);
        lastError = error;

        double PID = getKp() * error + getKi() * integral + getKd() * change;

        double rot = FtcUtil.motorScale(PID);
        this.move(pow, angle, rot);
    }

    /*
     * Constant values for PID.
     */
    double getKp() {
        return -0.02;
    }

    double getKi() {
        return 0.0;
    }

    double getKd() {
        return 0.0;
    }
}