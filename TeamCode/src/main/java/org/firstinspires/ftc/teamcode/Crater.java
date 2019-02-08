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

@Autonomous(name="Crater", group="Autonomous")
public class Crater extends MasterAuto {

    static double INITIAL_D = 0.5;                          // after we drop, we move this far fwd

    static int ANGLE_GOLD = 45;                             // rotate this much to face gold

    static double STRT_GOLD_D = 0.5 * Math.sqrt(2);         // go this far to hit gold
    static double DIAG_GOLD_D = 1;                          // then go back the same distance

    static int ANGLE_WALL = 90;                             // face here relative to start
                                                            // before going to wall

    static double LANDR_TO_WALL_D = 1.5 * Math.sqrt(2);     // drive this far from sample site
    static double WALL_TO_DEPOT_D = 1;                      // drive this far straight to depot

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

        driveAC(INITIAL_D, 0.25);

        int gpos = tfod(5);


        if(gpos == 0) { // LEFT DIAGONAL
            turnAC(-ANGLE_GOLD, 0.25);
            // turn to face the gold

            driveAC(DIAG_GOLD_D, 0.25);
            // gold has been hit

            driveAC(-DIAG_GOLD_D, 0.25);
            // we're back where we started, facing the blue wall

            turnAC(ANGLE_GOLD - ANGLE_WALL, 0.25);
            // we're facing along the lander

            driveAC(LANDR_TO_WALL_D, 0.25);
            // we're at the wall, facing it at 45 degrees

            turnAC(ANGLE_WALL - 135, 0.25);
            // we're at the wall, facing the depot
        }
        if(gpos == 1) { // MIDDLE
            driveAC(STRT_GOLD_D, 0.25);
            // gold has been hit

            driveAC(-STRT_GOLD_D, 0.25);
            // we're back where we started, facing the corner

            turnAC(-ANGLE_WALL, 0.25);
            // we're facing along the lander

            driveAC(LANDR_TO_WALL_D, 0.25);
            // we're at the wall, facing it at 45 degrees

            turnAC(ANGLE_WALL - 135, 0.25);
            // we're at the wall, facing the depot
        }
        if(gpos == 2) { // RIGHT DIAGONAL
            turnAC(ANGLE_GOLD, 0.25);
            // turn to face the gold

            driveAC(DIAG_GOLD_D, 0.25);
            // gold has been hit

            driveAC(-DIAG_GOLD_D, 0.25);
            // we're back where we started, facing the red wall

            turnAC(-ANGLE_GOLD - ANGLE_WALL, 0.25);
            // we're facing along the lander

            driveAC(LANDR_TO_WALL_D, 0.25);
            // we're at the wall, facing it at 45 degrees

            turnAC(ANGLE_WALL - 135, 0.25);
            // we're at the wall, facing the depot
        }

        driveAC(WALL_TO_DEPOT_D,0.25);
        // we're at the depot
        
        turnAC(ANGLE_DEPOT, 0.25);
        // angle ourselves so marker can be deployed

        /** deploy the marker */

        turnAC(180, 0.25);
        // we're at the depot, facing the crater

        driveAC(DEPO_TO_CRATR_D,1);
        // we're hopefully at the crater

        turnAC(ANGLE_CRATER, 0.25);
        // angle ourselves to throw something over the edge

        /** deploy something over the edge */

        halt();
    }
}