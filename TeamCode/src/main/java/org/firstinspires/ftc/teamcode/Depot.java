package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Depot", group="Autonomous")       // this file super is not depot
public class Depot extends MasterAuto {

    static double INITIAL_D = 0.5;                          // after we drop, we move this far fwd

    static int ANGLE_GOLD = 45;                             // rotate this much to face gold

    static double STRT_GOLD_D = 0.5 * Math.sqrt(2);         // go this far to hit gold
    static double DIAG_GOLD_D = 1;                          // then go back the same distance

    static int ANGLE_WALL = 90;                             // face here relative to start
                                                            // before going to wall

    static double LANDR_TO_WALL_D = 1.5 * Math.sqrt(2);     // drive this far from sample site
    static double WALL_TO_DEPOT_D = 1;                      // drive this far straight to depot

    static int ANGLE_DEPOT = 0;                             // rotate this much before marker drop

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

        driveAC(INITIAL_D, 0.25);

        int gpos = tfod(5);


        if(gpos == 2) { // RIGHT DIAGONAL
            turnAC(-ANGLE_GOLD, 0.25);
            // turn to face the gold

            driveAC(DIAG_GOLD_D, 0.25);
            // gold has been hit

            turnAC(ANGLE_GOLD, 0.25);
            // turn to face the depot

            driveAC(DIAG_GOLD_D, 0.25);
            // we're at the depot

            /** deploy */

            driveAC(-DIAG_GOLD_D, 0.25);
            // take a step back

            turnAC(-ANGLE_GOLD, 0.25);
            // turn back to lander

            driveAC(-DIAG_GOLD_D, 0.25);
            // we're at the lander

            turnAC(ANGLE_GOLD - ANGLE_WALL, 0.25);
            // we're facing along the lander

            driveAC(LANDR_TO_WALL_D, 0.25);
            // we're at the wall, facing it at 45 degrees

            turnAC(135 - ANGLE_WALL, 0.25);
            // we're at the wall, facing the crater
        }
        if(gpos == 1) { // MIDDLE
            driveAC(STRT_GOLD_D, 0.25);
            // gold has been hit

            driveAC(STRT_GOLD_D, 0.25);
            // go to depot

            /** deploy */

            driveAC(-STRT_GOLD_D, 0.25);
            // back a step

            driveAC(-STRT_GOLD_D, 0.25);
            // we're back where we started, facing the corner

            turnAC(ANGLE_GOLD - ANGLE_WALL, 0.25);
            // we're facing along the lander

            driveAC(LANDR_TO_WALL_D, 0.25);
            // we're at the wall, facing it at 45 degrees

            turnAC(135 - ANGLE_WALL, 0.25);
            // we're at the wall, facing the crater
        }
        if(gpos == 0) { // LEFT DIAGONAL
            turnAC(ANGLE_GOLD, 0.25);
            // turn to face the gold

            driveAC(DIAG_GOLD_D, 0.25);
            // gold has been hit

            turnAC(-ANGLE_GOLD, 0.25);
            // turn to face the depot

            driveAC(DIAG_GOLD_D, 0.25);
            // we're at the depot

            /** deploy */

            driveAC(-DIAG_GOLD_D, 0.25);
            // take a step back

            turnAC(ANGLE_GOLD, 0.25);
            // turn back to lander

            driveAC(-DIAG_GOLD_D, 0.25);
            // we're at the lander

            turnAC(-ANGLE_GOLD - ANGLE_WALL, 0.25);
            // we're facing along the lander

            driveAC(LANDR_TO_WALL_D, 0.25);
            // we're at the wall, facing it at 45 degrees

            turnAC(135 - ANGLE_WALL, 0.25);
            // we're at the wall, facing the crater
        }

        driveAC(WALL_TO_CRATR_D,1);
        // we're hopefully at the crater

        turnAC(ANGLE_CRATER, 0.25);
        // angle ourselves to throw something over the edge

        /** deploy something over the edge */

        halt();
    }
}