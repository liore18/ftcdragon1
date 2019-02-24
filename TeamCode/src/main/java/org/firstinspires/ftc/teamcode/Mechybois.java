package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static java.lang.Thread.sleep;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Mecanum", group="TeleOp")
public class Mechybois extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor lf = null;      // drivetrain
    private DcMotor rf = null;
    private DcMotor lb = null;
    private DcMotor rb = null;

    private DcMotor coll = null;    // intake
    private DcMotor coll_arm = null;

    private DcMotor lift = null;    // lift
    private Servo hook = null;

    private DcMotor fly = null;     // shooter
    private Servo gate = null;

    private TouchSensor touch = null;
    private TouchSensor touch2 = null;

    private TouchSensor ltouch = null;

    BNO055IMU gyro;

    boolean latchassist = false;
    int lockval = 0;
    float urgency = 0.01f;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        lf = hardwareMap.get(DcMotor.class, "lf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rf = hardwareMap.get(DcMotor.class, "rf");
        rb = hardwareMap.get(DcMotor.class, "rb");

        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);

        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        coll_arm = hardwareMap.get(DcMotor.class, "coll_arm");
        coll = hardwareMap.get(DcMotor.class, "coll");          // collection motor

        hook = hardwareMap.get(Servo.class, "hook");
        lift = hardwareMap.get(DcMotor.class, "lift");          // lift motor

        fly = hardwareMap.get(DcMotor.class, "fly");
        gate = hardwareMap.get(Servo.class, "gate");

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        coll.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        coll_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        coll_arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        touch = hardwareMap.touchSensor.get("touch");
        touch2 = hardwareMap.touchSensor.get("touch2");

        ltouch = hardwareMap.touchSensor.get("ltouch");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "gyro";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro = hardwareMap.get(BNO055IMU.class, "gyro");
        gyro.initialize(parameters);                                // get and set up gyro

        telemetry.addData("Status", "Let's roll.");          // tell the driver we're all set
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }


    /* CONTROLS LIST

    LATCH ASSIST
    G1 left stick button        ON/RESET
    G1 right stick button       OFF

    DRIVE
    G1 left stick y             STRAIGHT
    G1 left stick x             STRAFE
    G1 right stick x            ROTATE
    G1 X                        SLOW

    LIFT
    G1|G2 dpad up               LIFT UP
    G1|G2 dpad down             LIFT DOWN
    G1|G2 right bumper          LATCH
    G1|G2 left bumper           UNLATCH

    COLLECTION
    G2 right trigger            SWING OUT
    G2 left trigger             SWING IN
    G2 A                        COLLECT
    G2 B                        EJECT

    SHOOTER
    G2 X                        FLYWHEEL TOGGLE
    G2 Y                        GATE UP MOMENTARY

     */

    boolean xpressed = false;

    boolean flywheel = false;

    @Override
    public void loop() {

        //region latch assist
        if(gamepad1.left_stick_button) {
            lockval = gg();
            latchassist = true;
        } else if(gamepad1.right_stick_button) {
            latchassist = false;
        }

        float AC;
        float con;
        if(latchassist) {
            if(!ltouch.isPressed()) con = -0.12f; else con = 0;
            int current = gg();
            float d = lockval - current;
            AC = d * urgency;
        } else {
            con = 0;
            AC = 0;
        }
        //endregion

        //region drivetrain power
        float drive = scaleInput(-gamepad1.left_stick_y);
        float strafe = scaleInput(gamepad1.left_stick_x);
        float rotate = scaleInput(gamepad1.right_stick_x);

        telemetry.addData("drive", + drive);
        telemetry.addData("strafe", + strafe);
        telemetry.addData("rotate", + rotate);

        if(Math.abs(drive) < 0.05f) drive = 0.0f;
        if(Math.abs(strafe) < 0.05f) strafe = 0.0f;
        if(Math.abs(rotate) < 0.05f) rotate = 0.0f;

        if(!gamepad1.x) {
            lf.setPower(Range.clip(drive + strafe + rotate - AC + con, -1.0, 1.0));
            lb.setPower(Range.clip(drive - strafe + rotate - AC + con, -1.0, 1.0));
            rf.setPower(Range.clip(drive - strafe - rotate + AC + con, -1.0, 1.0));
            rb.setPower(Range.clip(drive + strafe - rotate + AC + con, -1.0, 1.0));
        }
        else{
            lf.setPower(.2*Range.clip(drive + strafe + rotate - AC + con, -1.0, 1.0));
            lb.setPower(.2*Range.clip(drive - strafe + rotate - AC + con, -1.0, 1.0));
            rf.setPower(.2*Range.clip(drive - strafe - rotate + AC + con, -1.0, 1.0));
            rb.setPower(.2*Range.clip(drive + strafe - rotate + AC + con, -1.0, 1.0));
        }
        //endregion

        //region lift
        if((gamepad2.dpad_up || gamepad1.dpad_up) && !touch.isPressed()){         //up
            lift.setPower(1);
        } else if((gamepad2.dpad_down || gamepad1.dpad_down) && !touch2.isPressed()){         //down
           lift.setPower(-1);
        } else {
            lift.setPower(0);
        }

        if(gamepad2.right_bumper || gamepad1.right_bumper){
            hook.setPosition(1);
        } else if (gamepad2.left_bumper || gamepad1.left_bumper){
            hook.setPosition(0);
        }
        //endregion

        //region collection
        if(gamepad2.left_trigger > 0.05 || gamepad2.right_trigger > 0.05){
            coll_arm.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
        } else {
            coll_arm.setPower(0);
        }

        if(gamepad2.a){
            coll.setPower(0.5);
        } else if(gamepad2.b){
            coll.setPower(-0.5);
        } else { coll.setPower(0); }
        //endregion

        //region colorSensor
        // values is a reference to the hsvValues array.
        float[] hsvValues = new float[3];
        final float values[] = hsvValues;

        // Get a reference to our sensor object.
        colorSensor = hardwareMap.get(NormalizedColorSensor.class , "colorSensor");

        View relativeLayout;

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        relativeLayout.setBackgroundColor(Color.WHITE); //sets background color to white initially

        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        // Read the sensor
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        /** Use telemetry to display feedback on the driver station. We show the conversion
         * of the colors to hue, saturation and value, and display the the normalized values
         * as returned from the sensor.
         * @see <a href="http://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html">HSV</a>*/

        Color.colorToHSV(colors.toColor(), hsvValues);
        telemetry.addLine()
                .addData("H", "%.3f", hsvValues[0])
                .addData("S", "%.3f", hsvValues[1])
                .addData("V", "%.3f", hsvValues[2]);
        telemetry.addLine()
                .addData("a", "%.3f", colors.alpha)
                .addData("r", "%.3f", colors.red)
                .addData("g", "%.3f", colors.green)
                .addData("b", "%.3f", colors.blue);

        /** We also display a conversion of the colors to an equivalent Android color integer.
         * @see Color */
        int color = colors.toColor();
        telemetry.addLine("raw Android color: ")
                .addData("a", "%02x", Color.alpha(color))
                .addData("r", "%02x", Color.red(color))
                .addData("g", "%02x", Color.green(color))
                .addData("b", "%02x", Color.blue(color));

        // Balance the colors. The values returned by getColors() are normalized relative to the
        // maximum possible values that the sensor can measure. For example, a sensor might in a
        // particular configuration be able to internally measure color intensity in a range of
        // [0, 10240]. In such a case, the values returned by getColors() will be divided by 10240
        // so as to return a value it the range [0,1]. However, and this is the point, even so, the
        // values we see here may not get close to 1.0 in, e.g., low light conditions where the
        // sensor measurements don't approach their maximum limit. In such situations, the *relative*
        // intensities of the colors are likely what is most interesting. Here, for example, we boost
        // the signal on the colors while maintaining their relative balance so as to give more
        // vibrant visual feedback on the robot controller visual display.
        float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
        colors.red   /= max;
        colors.green /= max;
        colors.blue  /= max;
        color = colors.toColor();

        telemetry.addLine("normalized color:  ")
                .addData("a", "%02x", Color.alpha(color))
                .addData("r", "%02x", Color.red(color))
                .addData("g", "%02x", Color.green(color))
                .addData("b", "%02x", Color.blue(color));
        telemetry.update();

        // convert the RGB values to HSV values.
        Color.RGBToHSV(Color.red(color), Color.green(color), Color.blue(color), hsvValues);

        // change the background color to match the color detected by the RGB sensor.
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.
        relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));

        //endregion

        //region shooter



        if(gamepad2.y) {
            gate.setPosition(0);
        } else {
            gate.setPosition(1);
        }

        if(gamepad2.x) {
            if(!xpressed) {
                flywheel = !flywheel;
            }
            xpressed = true;
        } else {
            xpressed = false;
        }

        if(flywheel) {
            fly.setPower(1);
        } else {
            fly.setPower(0);
        }

        //endregion

        //region telemetry
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("rb power", + rb.getPower());
        telemetry.addData("rf power", + rf.getPower());
        telemetry.addData("lf power", + lf.getPower());
        telemetry.addData("lb power", + lb.getPower());

        telemetry.addData("rb pos", + rb.getCurrentPosition());
        telemetry.addData("rf pos", + rf.getCurrentPosition());
        telemetry.addData("lf pos", + lf.getCurrentPosition());
        telemetry.addData("lb pos", + lb.getCurrentPosition());

        telemetry.addData("touch", ltouch.isPressed());

        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.update();
        //endregion
    }

    public void shoot(double mult) {
        double os = getRuntime();

        while (fly.getPower() < 1.0) {
            fly.setPower((getRuntime() - os) * mult);
        }
        for(int i = 0; i < 1000; i++)
            fly.setPower(1);

        fly.setPower(0);
    }

    public void collect() {
        /*

        when we press a button:
            check where the arm is in its range of motion

            if arm is up:
                power motor until some point
                turn on collection motor

            if arm is down:




         */
    }


    @Override
    public void stop() {
    }

    float scaleInput(float in) {
        float out = in*in;
        if (in < 0)
            out = -out;
        return(out);
    }

    /**
     * @return the current gyro reading as an int from 0 to 360 degrees
     */
    int gg() {
        int gyroAngle = (int)gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        return gyroAngle;
    }
}
