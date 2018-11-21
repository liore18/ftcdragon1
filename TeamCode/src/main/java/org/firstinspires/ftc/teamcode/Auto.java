package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Autonomous", group="Autonomous")
public class Auto extends LinearOpMode {

    // constants to help calculate how far we're moving

    static final double TICKS_PER_ROTATION = 1440.0;
    static final double WHEEL_DIAMETER = 3.543;  // bad units

    static final double TICKS_PER_INCH = (TICKS_PER_ROTATION) / (WHEEL_DIAMETER * Math.PI);
    static final double TICKS_PER_TILE = TICKS_PER_INCH * 24;
    static final double WHEEL_SPAN = 10.86614; // bad units

    // variables.

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lb = null;
    private DcMotor rb = null;

    private DcMotor[] right;
    private DcMotor[] left;

    private DcMotor coll = null;
    private DcMotor lift = null; // p2rs
    private DcMotor winch = null;

    private Servo dump = null; // p2rt
    private Servo grip = null;

    private CRServo ex = null;
    private Servo dr = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "will now init");
        telemetry.update();

        initialize();

        telemetry.addData("Status", "will now reset");
        telemetry.update();

        reset();

        telemetry.addData("Status", "Nominal");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Status", "Ready to drive");
        telemetry.update();

        //          PUT ALL THE CODE HERE
        //  use drive(distance, power) to move forward or backward (distance is in field tiles)
        //  use pivot(angle, power) to turn while stopped
        //



        rf.setPower(0.2);
        rf.setTargetPosition(100000);

        rb.setPower(0.2);
        rb.setTargetPosition(100000);

        lf.setPower(0.2);
        lf.setTargetPosition(100000);

        lb.setPower(0.2);
        lb.setTargetPosition(100000);

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
        //drive(0.5, 0.01);

        //pivot(90, 0.3);
        halt();
    }

    public void initialize() {
        rf = hardwareMap.get(DcMotor.class, "rf");         // the
        rb = hardwareMap.get(DcMotor.class, "rb");         // four
        lf = hardwareMap.get(DcMotor.class, "lf");        // go into
        lb = hardwareMap.get(DcMotor.class, "lb");        // two arrays

        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);
        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);


        // Tell the driver that initialization is complete.
        ex = hardwareMap.get(CRServo.class, "ex");              // extend marker
        dr = hardwareMap.get(Servo.class, "dr");                // drop marker
    }

    private void reset() {
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void halt() {
        rf.setPower(0);
        rb.setPower(0);
        lf.setPower(0);
        lb.setPower(0);
    }

    public void drive(double distance, double pwr) {
        reset();
        int target = (int)(distance * TICKS_PER_TILE);

        pwr = Math.abs(pwr);

        if(target < 0)
            pwr = -pwr;
        for(DcMotor d : left) {
            d.setTargetPosition(target);
            d.setPower(pwr);
        }
        for(DcMotor d : right) {
            d.setTargetPosition(target);
            d.setPower(pwr);
        }

        while (opModeIsActive() && left[0].isBusy()) {
            telemetry.addData("position", left[0].getCurrentPosition());
            telemetry.update();
        }
        halt();
    }

    public void drive(double pwr, double distance, double arc) {    // NOT IMPLEMENTED

    }

    public void pivot(double angle, double pwr) {
        reset();
        double distance = Math.PI * WHEEL_SPAN * angle / 360;
        int target = (int)(distance * TICKS_PER_INCH);
        pwr = Math.abs(pwr);

        if(target < 0)
            pwr = -pwr;

        for(DcMotor d : left) {
            d.setTargetPosition(target);
            d.setPower(pwr);
        }
        for(DcMotor d : right) {
            d.setTargetPosition(-target);
            d.setPower(-pwr);
        }

        while (opModeIsActive() && left[0].isBusy()) {

        }
        halt();
    }
}
