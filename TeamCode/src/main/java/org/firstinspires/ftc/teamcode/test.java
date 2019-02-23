package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="test", group="Autonomous")
public class test extends MasterAuto {

    private ElapsedTime runtime = new ElapsedTime();

    private Servo drop = null;

    @Override
    public void runOpMode() {

        initialize();
        reset();

        waitForStart();

        turnAC_debug(90, 0.25);
        sleep(1000);
        turnAC(-90, 0.25);

        halt();
    }
}