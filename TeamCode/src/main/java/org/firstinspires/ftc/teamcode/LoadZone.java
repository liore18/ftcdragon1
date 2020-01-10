package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name="LoadZone", group="Autonomous")
public class LoadZone extends MasterAuto2020 {



    @Override
    public void runOpMode() {

        //region initialization
        initialize();

        reset();

        strafeAC(1,.25);
        driveAC(2,.25);
        movePlat();
        driveAC(2,-.25);
        releasePlat();
        strafeAC(1,-.25);



        halt();
    }
}