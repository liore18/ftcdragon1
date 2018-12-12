package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
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

@TeleOp(name="Tank Drive", group="TeleOp")
public class TankDrive extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lb = null;
    private DcMotor rb = null;

    private DcMotor coll = null;
    private DcMotor lift = null; // p2rs
    private DcMotor winch = null;

    private Servo dump = null; // p2rt
    private Servo grip = null;

    private CRServo ex = null;
    private Servo dr = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        lf = hardwareMap.get(DcMotor.class, "lf");         // the
        lb = hardwareMap.get(DcMotor.class, "lb");         // four

         // motors
        rf = hardwareMap.get(DcMotor.class, "rf");        // go into
        rb = hardwareMap.get(DcMotor.class, "rb");        // two array

        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);

        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        ex = hardwareMap.get(CRServo.class, "ex");              // extend marker
        dr = hardwareMap.get(Servo.class, "dr");                // drop marker

        coll = hardwareMap.get(DcMotor.class, "coll");                   // collection motor
        lift = hardwareMap.get(DcMotor.class, "lift"); // lift motor

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
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        leftPower = Range.clip(drive + turn, -1.0, 1.0);
        rightPower = Range.clip(drive - turn, -1.0, 1.0);

        // Send calculated power to wheels

            lf.setPower(leftPower);
            lb.setPower(leftPower);

            rf.setPower(rightPower);
            rb.setPower(rightPower);

        //ex.setPower(0.5);
//extend
        //if(gamepad1.x)
            //ex.setPower(0.45);
        //else
           // ex.setPower(-0.6);
//drop
       // if(gamepad1.right_bumper)
          //  dr.setPosition(0.4);
       // if(gamepad1.left_bumper)
           // dr.setPosition(1.0);
//run collection
        if(gamepad1.right_trigger > 0.5)
            coll.setPower(1);
        else
            coll.setPower(0);
//lift
        if(gamepad1.a){         //up
            lift.setPower(1);
        }else
            lift.setPower(0);

        if(gamepad1.b){         //down
           lift.setPower(-1);
        }else
            lift.setPower(0);



        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.update();
    }

    @Override
    public void stop() {
    }

}
