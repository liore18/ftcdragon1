package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Depot Full Send", group="Autonomous")
public class AutoGunDepot extends LinearOpMode {

    // constants to help calculate how far we're moving

    static final double TICKS_PER_ROTATION = 2240.0 / 2;
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

    private DcMotor lift = null; // p2rs
    private Servo claw = null;

    private CRServo ex = null;
    private Servo dr = null;

    @Override
    public void runOpMode() {

        initialize();

        reset();

        telemetry.addData("Status", "Nominal");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Status", "Driving");
        telemetry.update();

        //          PUT ALL THE CODE HERE
        //  use drive(distance, power) to move forward or backward (distance is in field tiles)
        //  use pivot(angle, power) to turn while stopped
        //
        drive(2.475,.7);
        dropmarker();
        pivot(45,.7);
        pivot(-1,0.05);
        drive(-5,1);
        halt();
    }

    private void initialize() {
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

        lift = hardwareMap.get(DcMotor.class, "lift");
        claw = hardwareMap.get(Servo.class, "claw");
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
        reset();
        halt();
    }

    public void drive(double pwr, double distance, double arc) {    // NOT IMPLEMENTED
        reset();
        double arclength = Math.PI * WHEEL_SPAN * arc / 180;

        int targetturn = (int)(arclength * TICKS_PER_INCH);

        int targetdrive = (int)(distance * TICKS_PER_TILE);
        pwr = Math.abs(pwr);

        rf.setTargetPosition(targetdrive + targetturn);
        rb.setTargetPosition(targetdrive + (int)(targetturn * .5547));
        lf.setTargetPosition(targetdrive - targetturn);
        lb.setTargetPosition(targetdrive - (int)(targetturn * .5547));

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

    public void pivot(double angle, double pwr) {
        reset();
        double distance = Math.PI * (WHEEL_SPAN) * angle / 255; // remember learning s = r*theta? it's back to haunt you.
        int target = (int)(distance * TICKS_PER_INCH);
        pwr = Math.abs(pwr);

        rf.setTargetPosition(target);
        rb.setTargetPosition((int)(target * .5547));
        lf.setTargetPosition(-target);
        lb.setTargetPosition(-(int)(target * .5547));
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

    public void pivottheotherone(double angle, double pwr) {
        reset();
        double distance = Math.PI * (WHEEL_SPAN/2) * angle / 180; // remember learning s = r*theta? it's back to haunt you.
        int target = (int)(distance * TICKS_PER_INCH);
        pwr = Math.abs(pwr);

        rf.setTargetPosition(target);
        rb.setTargetPosition(0);
        lf.setTargetPosition(-target);
        lb.setTargetPosition(0);
        rf.setPower(pwr);
        rb.setPower(0);
        lf.setPower(pwr);
        lb.setPower(0);

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

    public void dropmarker() {
        ex.setPower(1.0);
        sleep(5000);
        ex.setPower(0.2);
        dr.setPosition(1.0);
        sleep(4000);
        dr.setPosition(.4);
        sleep(4000);
        ex.setPower(-1.0);
        sleep(5000);
    }


    public void lift() {
        lift.setPower(1.0);
        sleep(3000);
        lift.setPower(0);
        claw.setPosition(1.0);
    }


    
}
