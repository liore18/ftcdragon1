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
    static final double WHEEL_DIAMETER = 4.0 ; // centimetres, because clearly they're superior
    static final double TICKS_PER_TILE = (TICKS_PER_ROTATION * 60.96) / (WHEEL_DIAMETER * 3.1415);

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

        reset();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Status", "Nominal");
        telemetry.update();
    }

    private void reset() {
        for(DcMotor d : left) {
            d.setDirection(DcMotor.Direction.REVERSE);
            d.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            d.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            d.setPower(0);
        }
        for(DcMotor d : right) {
            d.setDirection(DcMotor.Direction.FORWARD);
            d.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            d.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            d.setPower(0);
        }
    }

    private void halt() {
        for(DcMotor d : left) {
            d.setPower(0);
        }
        for(DcMotor d : right) {
            d.setPower(0);
        }
    }

    public void drive(double pwr, double distance) {
        reset();
        int target = (int)(distance * TICKS_PER_TILE);
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

        }
        halt();
    }

    public void drive(double pwr, double distance, double arc) {

    }

    public void pivot(double angle) {

    }
}
