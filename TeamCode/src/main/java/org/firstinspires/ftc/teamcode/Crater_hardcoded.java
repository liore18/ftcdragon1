package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name="ch", group="Autonomous")
public class Crater_hardcoded extends MasterAuto {

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

        driveAC(0.5, 0.25);

        int gpos = tfod(5);

        if(gpos == 1) {
            driveAC(0.5 * Math.sqrt(2), 0.25);
            // gold has been hit

            driveAC(-0.5 * Math.sqrt(2), 0.25);
            // we're back where we started, facing the corner

            turnAC(-90, 0.25);
            // we're facing along the lander

            driveAC(1.5 * Math.sqrt(2), 0.25);
            // we're at the wall, facing it at 45 degrees

            turnAC(-45, 0.25);
            // we're at the wall, facing the depot
        }
        if(gpos == 0) {
            turnAC(-45, 0.25);
            // turn to face the gold

            driveAC(1, 0.25);
            // gold has been hit

            driveAC(-1, 0.25);
            // we're back where we started, facing the blue wall

            turnAC(-45, 0.25);
            // we're facing along the lander

            driveAC(1.5 * Math.sqrt(2), 0.25);
            // we're at the wall, facing it at 45 degrees

            turnAC(-45, 0.25);
            // we're at the wall, facing the depot

        }
        if(gpos == 2) {
            turnAC(45, 0.25);
            // turn to face the gold

            driveAC(1, 0.25);
            // gold has been hit

            driveAC(-1, 0.25);
            // we're back where we started, facing the red wall

            turnAC(-135, 0.25);
            // we're facing along the lander

            driveAC(1.5 * Math.sqrt(2), 0.25);
            // we're at the wall, facing it at 45 degrees

            turnAC(-45, 0.25);
            // we're at the wall, facing the depot

        }

        driveAC(1,0.25);
        // we're at the depot

        /** deploy the marker */

        turnAC(180, 0.25);
        // we're at the depot, facing the crater

        driveAC(3,1);
        // we're hopefully at the crater

        /** deploy something over the edge */

        halt();
    }
}