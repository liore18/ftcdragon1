package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Disabled
@Autonomous(name="Crater", group="Autonomous")
public class Crater extends MasterAuto2020 {

    static double INITIAL_D = 0.5;                          // after we drop, we move this far fwd

    static int ANGLE_SAMPLE = 22;

    static int ANGLE_GOLD = 45;                             // rotate this much to face gold

    static double STRT_GOLD_D = 0.45 * Math.sqrt(2);        // go this far to hit gold
    static double DIAG_GOLD_D = 0.8;                        // then go back the same distance

    static int ANGLE_WALL = 85;
    // face here relative to start
                                                            // before going to wall

    static double LANDR_TO_WALL_D = 1.5 * Math.sqrt(2) - 0.5;     // drive this far from sample site
    static double WALL_TO_DEPOT_D = 2.5;                    // drive this far straight to depot

    static int ANGLE_DEPOT = 0;                             // rotate this much before marker drop

    static double DEPO_TO_CRATR_D = 3;                      // drive this far straight to crater

    static int ANGLE_CRATER = 0;                            // rotate this much once at crater


    @Override
    public void runOpMode() {

        //region initialization
        initialize();

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

        lift();
        // now more code


        int gpos = tfod2(5);

        driveAC(INITIAL_D, 0.25);


        if(gpos == 2) { // RIGHT DIAGONAL
            turnAC(-ANGLE_GOLD, 0.25);
            // turn to face the gold

            driveAC(DIAG_GOLD_D, 0.25);
            // gold has been hit

            driveAC(-DIAG_GOLD_D, 0.25);
            // we're back where we started, facing the blue wall

            turnAC(ANGLE_GOLD + ANGLE_WALL, 0.25);
            // we're facing along the lander
        }
        if(gpos == 1) { // MIDDLE
            driveAC(STRT_GOLD_D, 0.25);
            // gold has been hit

            driveAC(-STRT_GOLD_D, 0.25);
            // we're back where we started, facing the corner

            turnAC(ANGLE_WALL, 0.25);
            // we're facing along the lander
        }
        if(gpos == 0) { // LEFT DIAGONAL
            turnAC(ANGLE_GOLD, 0.25);
            // turn to face the gold

            driveAC(DIAG_GOLD_D, 0.25);
            // gold has been hit

            driveAC(-DIAG_GOLD_D, 0.25);
            // we're back where we started, facing the red wall

            turnAC(ANGLE_WALL - ANGLE_GOLD, 0.25);
            // we're facing along the lander
        }

        drive(LANDR_TO_WALL_D, 0.5);
        // we're at the wall, facing it at 45 degrees

        turnAC(ANGLE_WALL - 45, 0.25);
        // we're at the wall, facing the depot

        alignWall(3);  // STRAFE UNTIL WALL

        driveAC(WALL_TO_DEPOT_D,0.7);
        // we're at the depot

        dropmarker();

        driveAC(-0.25, 0.7); // GIVE US SPACE TO TURN

        /*turnAC(-180, 0.25);                   // THIS IS RISKY
        // we're at the depot, facing the crater

        driveAC(0.25, 1);*/

        /** deploy something over the edge */

        halt();
    }
}