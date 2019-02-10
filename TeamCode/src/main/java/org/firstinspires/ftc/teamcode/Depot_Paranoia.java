package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Depot Spin After", group="Autonomous")       // this file super is not depot
public class Depot_Paranoia extends MasterAuto {

    static double INITIAL_D = 0.5;                          // after we drop, we move this far fwd

    static int ANGLE_SAMPLE = 22;

    static int ANGLE_GOLD = 45;                             // rotate this much to face gold

    static double STRT_GOLD_D = 1 * Math.sqrt(2);         // go this far to hit gold
    static double DIAG_GOLD_D = 1;                          // then go back the same distance

    static int ANGLE_WALL = 70;                             // face here relative to start
                                                            // before going to wall

    static double LANDR_TO_WALL_D = 1.5 * Math.sqrt(2) - 0.5;     // drive this far from sample site

    static double WALL_TO_CRATR_D = 3;                      // drive this far straight to crater

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

            turnAC(-90, 0.25);
            // our back is to the depot

            driveAC(-2, 0.25);
            deploy();
            driveAC(2, 0.25);
        }
        if(gpos == 1) { // MIDDLE
            driveAC(STRT_GOLD_D, 0.25);
            // gold has been hit

            driveAC(-STRT_GOLD_D, 0.25);
            // we're back where we started, facing the corner

        }
        if(gpos == 0) { // LEFT DIAGONAL
            turnAC(ANGLE_GOLD, 0.25);
            // turn to face the gold

            driveAC(DIAG_GOLD_D, 0.25);
            // gold has been hit

            driveAC(-DIAG_GOLD_D, 0.25);
            // we're back where we started, facing the red wall

            turnAC(90, 0.25);
            // our back is to the depot

            driveAC(-2, 0.25);
            deploy();
            driveAC(2, 0.25);
        }

        /*driveAC(WALL_TO_CRATR_D,1);
        // we're hopefully at the crater

        turnAC(ANGLE_CRATER, 0.25);
        // angle ourselves to throw something over the edge

        /** deploy something over the edge */

        driveAC(1, 0.5);

        halt();
    }

    void deploy() {
        turnAC(90, 0.25);
        turnAC(90, 0.25);

        dropmarker();

        turnAC(90, 0.25);
        turnAC(90, 0.25);
    }
}