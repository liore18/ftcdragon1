package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

@Autonomous(name="AUTO TESTING", group="Autonomous")
public class AUTO_TESTING extends LinearOpMode {

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
    @Override
    public void runOpMode() {

        initialize();

        reset();

        waitForStart();

        drive(10,.5);   // move 6 inches forward
    }

    private void initialize() {
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

    private void drive(double distance, double pwr) {
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
        halt();
        reset();
    }
}
