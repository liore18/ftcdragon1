package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

@Disabled
@Autonomous(name="Depot FS", group="Autonomous")
public class MasterAuto extends LinearOpMode {
    //Tfod constants
    public static final String VUFORIA_KEY = "AXDMU6L/////AAABmYsje6g+d0FouarmMJSceCUvoPLsXYHB38V7+MVCV//rzuYmaMR0aeKY+X1gyKROXD2HP/yqTdMoGKjNifE0TLgN3fUlxqF8CAejftyRLJXX7t1xBrivJKRDgDbQrX6I+6xe2ZcfInF2KnfQHOrlMh/i7M4RU6vzkIwKIzCwkV/SaMxAyYWpEngCIK+3ZelwN2uVIc0nXFNEXI2qVTaiAb7ffvbqzCBcxXrxCzbahSso5A/fD9f6FGsyMvVTQUzRaybT473gX+RJ1nPHyqjTscffYVyBGl0sAQ259VwLGwM+FE+ymehKO1shL9s1ITfaZaRdSWxzxvdS/e5xaavoXEw3ylD16GUnclpvw1s/ts7y";
    public static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    // constants to help calculate how far we're moving
    static final double TICKS_PER_ROTATION = 2240.0 / 2;
    static final double WHEEL_DIAMETER = 3.543;  // bad units

    static final double TICKS_PER_INCH = (TICKS_PER_ROTATION) / (WHEEL_DIAMETER * Math.PI);
    static final double TICKS_PER_TILE = TICKS_PER_INCH * 24;
    static final double WHEEL_SPAN = 10.86614; // bad units


    BNO055IMU gyro;
    // variables.

    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    public ElapsedTime runtime = new ElapsedTime();

    public DcMotor lf = null;
    public DcMotor rf = null;
    public DcMotor lb = null;
    public DcMotor rb = null;

    public DcMotor lift = null; // p2rs
    public Servo hook = null;

    public CRServo ex = null;
    public Servo dr = null;

    public TouchSensor touch = null;
    public TouchSensor touch2 = null;

    public DcMotor coll = null;
    public DcMotor coll_arm = null;
    public DcMotor coll_lift = null;

    public void runOpMode() {}

    public void initialize() {
        rf = hardwareMap.get(DcMotor.class, "rf");
        rb = hardwareMap.get(DcMotor.class, "rb");
        lf = hardwareMap.get(DcMotor.class, "lf");
        lb = hardwareMap.get(DcMotor.class, "lb");

        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);
        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);

        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Tell the driver that initialization is complete.
        lift = hardwareMap.get(DcMotor.class, "lift");
        hook = hardwareMap.get(Servo.class, "hook");

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        touch = hardwareMap.touchSensor.get("touch");
        touch2 = hardwareMap.touchSensor.get("touch2");

        coll_lift = hardwareMap.get(DcMotor.class, "coll_lift");
        coll_arm = hardwareMap.get(DcMotor.class, "coll_arm");
        coll = hardwareMap.get(DcMotor.class, "coll");

        hook.setPosition(0.0);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "gyro";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro = hardwareMap.get(BNO055IMU.class, "gyro");
        gyro.initialize(parameters);

        sleep(1000);
    }

    public void reset() {
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void halt() {
        rf.setPower(0);
        rb.setPower(0);
        lf.setPower(0);
        lb.setPower(0);
    }

    public void drive(double distance, double pwr) {
        reset();
        int target = (int) (distance * TICKS_PER_TILE);

        pwr = Math.abs(pwr);

        rf.setTargetPosition(target);
        rb.setTargetPosition(target);
        lf.setTargetPosition(target);
        lb.setTargetPosition(target);

        rf.setPower(pwr);
        rb.setPower(pwr);
        lf.setPower(pwr);
        lb.setPower(pwr);

        while (opModeIsActive() && rf.isBusy()) {
            telemetry.addData("rfpos", rf.getCurrentPosition());
            telemetry.addData("rbpos", rb.getCurrentPosition());
            telemetry.addData("lfpos", lf.getCurrentPosition());
            telemetry.addData("lbpos", lb.getCurrentPosition());

            telemetry.addData("rfpow", rf.getPower());
            telemetry.addData("rbpow", rb.getPower());
            telemetry.addData("lfpow", lf.getPower());
            telemetry.addData("lbpow", lb.getPower());
            telemetry.update();
        }
        halt();
        reset();
    }

    /**
     * @param distance to move forward in tiles
     * @param pwr      to give the motors, from 0.0 to 1.0
     * @param ramp     the distance in inches to accelerate linearly through
     */
    public void drive(double distance, double pwr, int ramp) {
        reset();
        int target = (int) (distance * TICKS_PER_TILE);

        double fpwr = Math.abs(pwr);

        rf.setTargetPosition(target);
        rb.setTargetPosition(target);
        lf.setTargetPosition(target);
        lb.setTargetPosition(target);

        while (opModeIsActive() && rf.isBusy()) {

            if (rf.getCurrentPosition() >= ramp * TICKS_PER_TILE / 24) {
                pwr = fpwr;
            } else {
                pwr = fpwr * (rf.getCurrentPosition() + 50) / (ramp * TICKS_PER_TILE / 24);
            }

            rf.setPower(pwr);
            rb.setPower(pwr);
            lf.setPower(pwr);
            lb.setPower(pwr);

            telemetry.addData("rfpos", rf.getCurrentPosition());
            telemetry.addData("rbpos", rb.getCurrentPosition());
            telemetry.addData("lfpos", lf.getCurrentPosition());
            telemetry.addData("lbpos", lb.getCurrentPosition());

            telemetry.addData("rfpow", rf.getPower());
            telemetry.addData("rbpow", rb.getPower());
            telemetry.addData("lfpow", lf.getPower());
            telemetry.addData("lbpow", lb.getPower());
            telemetry.update();
        }
        reset();
        halt();
    }

    public void drive(double distance, double pwr, double arc) {    // NOT IMPLEMENTED
        reset();
        double arclength = Math.PI * WHEEL_SPAN * arc / 180;

        int targetturn = (int) (arclength * TICKS_PER_INCH);

        int targetdrive = (int) (distance * TICKS_PER_TILE);
        pwr = Math.abs(pwr);

        rf.setTargetPosition(targetdrive + targetturn);
        rb.setTargetPosition(targetdrive + (int) (targetturn * .5547));
        lf.setTargetPosition(targetdrive - targetturn);
        lb.setTargetPosition(targetdrive - (int) (targetturn * .5547));

        rf.setPower(pwr);
        rb.setPower(pwr);
        lf.setPower(pwr);
        lb.setPower(pwr);

        while (opModeIsActive() && rf.isBusy()) {
            telemetry.addData("rfpos", rf.getCurrentPosition());
            telemetry.addData("rbpos", rb.getCurrentPosition());
            telemetry.addData("lfpos", lf.getCurrentPosition());
            telemetry.addData("lbpos", lb.getCurrentPosition());

            telemetry.addData("rfpow", rf.getPower());
            telemetry.addData("rbpow", rb.getPower());
            telemetry.addData("lfpow", lf.getPower());
            telemetry.addData("lbpow", lb.getPower());
            telemetry.update();
        }
        halt();
    }

    public void swivelpivot(double angle, double pwr) {
        reset();
        double distance = 0.8 * Math.PI * (WHEEL_SPAN) * angle / 255; // remember learning s = r*theta? it's back to haunt you.
        int target = (int) (distance * TICKS_PER_INCH);
        pwr = Math.abs(pwr);

        rf.setTargetPosition(target);
        rb.setTargetPosition((int) (target * .5547));
        lf.setTargetPosition(-target);
        lb.setTargetPosition(-(int) (target * .5547));
        rf.setPower(pwr);
        rb.setPower(pwr * .5547);
        lf.setPower(pwr);
        lb.setPower(pwr * .5547);

        while (opModeIsActive() && rf.isBusy()) {
            telemetry.addData("rfpos", rf.getCurrentPosition());
            telemetry.addData("rbpos", rb.getCurrentPosition());
            telemetry.addData("lfpos", lf.getCurrentPosition());
            telemetry.addData("lbpos", lb.getCurrentPosition());

            telemetry.addData("rfpow", rf.getPower());
            telemetry.addData("rbpow", rb.getPower());
            telemetry.addData("lfpow", lf.getPower());
            telemetry.addData("lbpow", lb.getPower());
            telemetry.update();
        }
        halt();
    }

    public void pivot(double angle, double pwr) {
        reset();
        double distance = Math.PI * (WHEEL_SPAN / 2) * angle / 180; // remember learning s = r*theta? it's back to haunt you.
        int target = (int) (distance * TICKS_PER_INCH);
        pwr = Math.abs(pwr);

        rf.setTargetPosition(target);
        rb.setTargetPosition(target);
        lf.setTargetPosition(-target);
        lb.setTargetPosition(-target);
        rf.setPower(pwr);
        rb.setPower(pwr);
        lf.setPower(pwr);
        lb.setPower(pwr);

        while (opModeIsActive() && rf.isBusy()) {
            telemetry.addData("rfD", rf.getCurrentPosition());
            telemetry.addData("rbD", rb.getCurrentPosition());
            telemetry.addData("lfD", lf.getCurrentPosition());
            telemetry.addData("lbD", lb.getCurrentPosition());

            telemetry.addData("rfP", rf.getPower());
            telemetry.addData("rbP", rb.getPower());
            telemetry.addData("lfP", lf.getPower());
            telemetry.addData("lbP", lb.getPower());
            telemetry.update();
        }
        halt();
    }

    public void dropmarker() {
        coll_arm.setPower(1);
        sleep(1200);
        coll_arm.setPower(0);
        sleep(1000);


        coll.setPower(-1);
        sleep(1000);
        coll.setPower(0);

        coll_arm.setPower(-1);
        sleep(2000);
        coll_arm.setPower(0);
    }

    public void floparm() {
        coll_arm.setPower(1);
        sleep(1200);
        coll_arm.setPower(0);
        sleep(1000);
    }

    public void lift() {
        lift.setPower(-1.0);
        while (!touch2.isPressed()) {
            telemetry.addData("Status", "LOWERING");
            telemetry.update();
        }
        lift.setPower(0);
        hook.setPosition(1.0);
        sleep(1000);
    }

    public void driveAC(double distance, double pwr) {
        reset();
        int target = (int) (distance * TICKS_PER_TILE);

        float urgency = 0.05f; // remember that we may end up more than 10 degrees off course.

        pwr = Math.abs(pwr);

        float initial = gg();
        float current;

        rf.setTargetPosition(target);
        rb.setTargetPosition(target);
        lf.setTargetPosition(target);
        lb.setTargetPosition(target);

        while (opModeIsActive() && rf.isBusy()) {
            //region telemetry
            telemetry.addData("rfpos", rf.getCurrentPosition());
            telemetry.addData("rbpos", rb.getCurrentPosition());
            telemetry.addData("lfpos", lf.getCurrentPosition());
            telemetry.addData("lbpos", lb.getCurrentPosition());

            telemetry.addData("rfpow", rf.getPower());
            telemetry.addData("rbpow", rb.getPower());
            telemetry.addData("lfpow", lf.getPower());
            telemetry.addData("lbpow", lb.getPower());
            telemetry.update();
            //endregion

            current = gg();
            float d = initial - current;

            rf.setPower(pwr - d * urgency);
            rb.setPower(pwr - d * urgency);
            lf.setPower(pwr + d * urgency);
            lb.setPower(pwr + d * urgency);
        }
        halt();
        reset();
    }

    public float gg() {
        float gyroAngle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        telemetry.addData("angle ", gyroAngle);
        telemetry.update();

        return gyroAngle;
    }
}