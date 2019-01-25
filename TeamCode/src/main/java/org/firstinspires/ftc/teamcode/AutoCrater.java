package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name="Crater", group="Autonomous")
public class AutoCrater extends LinearOpMode {
    //Tfod constants
    private static final String VUFORIA_KEY = "AXDMU6L/////AAABmYsje6g+d0FouarmMJSceCUvoPLsXYHB38V7+MVCV//rzuYmaMR0aeKY+X1gyKROXD2HP/yqTdMoGKjNifE0TLgN3fUlxqF8CAejftyRLJXX7t1xBrivJKRDgDbQrX6I+6xe2ZcfInF2KnfQHOrlMh/i7M4RU6vzkIwKIzCwkV/SaMxAyYWpEngCIK+3ZelwN2uVIc0nXFNEXI2qVTaiAb7ffvbqzCBcxXrxCzbahSso5A/fD9f6FGsyMvVTQUzRaybT473gX+RJ1nPHyqjTscffYVyBGl0sAQ259VwLGwM+FE+ymehKO1shL9s1ITfaZaRdSWxzxvdS/e5xaavoXEw3ylD16GUnclpvw1s/ts7y";
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    // constants to help calculate how far we're moving
    static final double TICKS_PER_ROTATION = 2240.0 / 2;
    static final double WHEEL_DIAMETER = 3.543;  // bad units

    static final double TICKS_PER_INCH = (TICKS_PER_ROTATION) / (WHEEL_DIAMETER * Math.PI);
    static final double TICKS_PER_TILE = TICKS_PER_INCH * 24;
    static final double WHEEL_SPAN = 10.86614; // bad units

    // variables.

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lb = null;
    private DcMotor rb = null;

    private DcMotor lift = null; // p2rs
    private Servo hook = null;

    private CRServo ex = null;
    private Servo dr = null;

    private TouchSensor touch = null;
    private TouchSensor touch2 = null;

    @Override
    public void runOpMode() {

        initialize();

        //region initialization
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        reset();

        lift.setPower(1.0);
        while(!touch.isPressed()) {
            telemetry.addData("Status", "RAISING");
            telemetry.update();
        }
        lift.setPower(0.0);

        telemetry.addData("Status", "Nominal");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Status", "Driving");
        telemetry.update();
        //endregion

        /**          PUT ALL THE CODE HERE
        *  use drive(distance, power) to move forward or backward (distance is in field tiles)
        *  use pivot(angle, power) to turn while stopped
        **/

        lift();
        drive(0.25,.5, 6);   // move 6 inches forward

        int gpos = 0;

        //region tfod
        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    gpos = 0;
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    gpos = 2;
                                } else {
                                    gpos = 1;
                                }
                            }
                        }
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
        //endregion

        if(gpos == 1) {
            drive(0.5 * Math.sqrt(2), 0.5, 6);
            drive(-0.5 * Math.sqrt(2), 0.5, 6);
            pivot(-90, 0.5);
            drive(1.5 * Math.sqrt(2), 0.5, 6);
            pivot(-45, 0.5);
        }
        if(gpos == 0) {
            pivot(-45, 0.5);
            drive(1, 0.5, 6);
            drive(-1, 0.5, 6);
            pivot(-45, 0.5);
            drive(1.5 * Math.sqrt(2), 0.5, 6);
            pivot(-45, 0.5);
        }
        if(gpos == 2) {
            pivot(45, 0.5);
            drive(1, 0.5, 6);
            drive(-1, 0.5, 6);
            pivot(-135, 0.5);
            drive(1.5 * Math.sqrt(2), 0.5, 6);
            pivot(-45, 0.5);
        }

        drive(3,0.7, 6);    // drive towards depot
        dropmarker();
        drive(-5,1, 6);
        halt();
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
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

        // Tell the driver that initialization is complete.
        lift = hardwareMap.get(DcMotor.class, "lift");
        hook = hardwareMap.get(Servo.class, "hook");

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        touch = hardwareMap.touchSensor.get("touch");
        touch2 = hardwareMap.touchSensor.get("touch2");

        hook.setPosition(0.0);
        sleep(1000);
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
        reset();
        halt();
    }

    /**
     * @param distance to move forward in tiles
     * @param pwr to give the motors, from 0.0 to 1.0
     * @param ramp the distance in inches to accelerate linearly through
     */
    private void drive(double distance, double pwr, int ramp) {
        reset();
        int target = (int)(distance * TICKS_PER_TILE);

        double fpwr = Math.abs(pwr);

        rf.setTargetPosition(target);
        rb.setTargetPosition(target);
        lf.setTargetPosition(target);
        lb.setTargetPosition(target);

        while (opModeIsActive() && rf.isBusy()) {

            if(rf.getCurrentPosition() >= ramp) {
                pwr = fpwr;
            } else {
                pwr = fpwr * rf.getCurrentPosition() / ramp;
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

    private void drive(double distance, double pwr, double arc) {    // NOT IMPLEMENTED
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

    private void swivelpivot(double angle, double pwr) {
        reset();
        double distance = 0.8*Math.PI * (WHEEL_SPAN) * angle / 255; // remember learning s = r*theta? it's back to haunt you.
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

    private void pivot(double angle, double pwr) {
        reset();
        double distance = Math.PI * (WHEEL_SPAN/2) * angle / 180; // remember learning s = r*theta? it's back to haunt you.
        int target = (int)(distance * TICKS_PER_INCH);
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

    private void dropmarker() {

    }

    private void lift() {
        lift.setPower(-1.0);
        while(!touch2.isPressed()) {
            telemetry.addData("Status", "LOWERING");
            telemetry.update();
        }
        lift.setPower(0);
        hook.setPosition(1.0);
        sleep(1000);
    }


    
}
