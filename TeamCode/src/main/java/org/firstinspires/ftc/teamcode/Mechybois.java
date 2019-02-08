package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lb = null;
    private DcMotor rb = null;

    private DcMotor coll = null;
    private DcMotor coll_lift = null;
    private DcMotor coll_arm = null;

    private DcMotor lift = null; // p2rs
    private Servo hook = null;

    private TouchSensor touch = null;
    private TouchSensor touch2 = null;


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

        coll_arm = hardwareMap.get(DcMotor.class, "coll_arm");
        coll_lift = hardwareMap.get(DcMotor.class, "coll_lift");
        coll = hardwareMap.get(DcMotor.class, "coll");          // collection motor

        hook = hardwareMap.get(Servo.class, "hook");
        lift = hardwareMap.get(DcMotor.class, "lift");          // lift motor

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        coll.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        coll_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        coll_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        touch = hardwareMap.touchSensor.get("touch");
        touch2 = hardwareMap.touchSensor.get("touch2");

        telemetry.addData("Status", "Let's roll.");          // tell the driver we're all set
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        coll.setPower(gamepad1.right_trigger);

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
            lf.setPower(Range.clip(drive + strafe + rotate, -1.0, 1.0));
            lb.setPower(Range.clip(drive - strafe + rotate, -1.0, 1.0));
            rf.setPower(Range.clip(drive - strafe - rotate, -1.0, 1.0));
            rb.setPower(Range.clip(drive + strafe - rotate, -1.0, 1.0));
        }
        else{
            lf.setPower(.2*Range.clip(drive + strafe + rotate, -1.0, 1.0));
            lb.setPower(.2*Range.clip(drive - strafe + rotate, -1.0, 1.0));
            rf.setPower(.2*Range.clip(drive - strafe - rotate, -1.0, 1.0));
            rb.setPower(.2*Range.clip(drive + strafe - rotate, -1.0, 1.0));
        }

        //liftyboie
        if((gamepad2.dpad_up || gamepad1.dpad_up) && !touch.isPressed()){         //up
            lift.setPower(1);
        } else if((gamepad2.dpad_down || gamepad1.dpad_down) && !touch2.isPressed()){         //down
           lift.setPower(-1);
        } else {
            lift.setPower(0);
        }

        if(gamepad2.left_trigger > 0.05 || gamepad2.right_trigger > 0.05){
            coll_arm.setPower(scaleInput(gamepad2.right_trigger) - scaleInput(gamepad2.left_trigger));
        } else {
            coll_arm.setPower(0);
        }

        if(gamepad2.x){
            coll_lift.setPower(1);
        } else if(gamepad2.y){
            coll_lift.setPower(-1);
        } else { coll_lift.setPower(0); }

        if(gamepad2.a){
            coll.setPower(1);
        } else if(gamepad2.b){
            coll.setPower(-1);
        } else { coll.setPower(0); }

        if(gamepad2.right_bumper || gamepad1.right_bumper){
            hook.setPosition(1);
        } else if (gamepad2.left_bumper || gamepad1.left_bumper){
            hook.setPosition(0);
        }


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("rb power", + rb.getPower());
        telemetry.addData("rf power", + rf.getPower());
        telemetry.addData("lf power", + lf.getPower());
        telemetry.addData("lb power", + lb.getPower());

        telemetry.addData("rfpos", rf.getCurrentPosition());
        telemetry.addData("rbpos", rb.getCurrentPosition());
        telemetry.addData("lfpos", lf.getCurrentPosition());
        telemetry.addData("lbpos", lb.getCurrentPosition());

        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.update();
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

}
