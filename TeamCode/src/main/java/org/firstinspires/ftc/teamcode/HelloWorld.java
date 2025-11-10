package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp //if you forget this it won't show up on the driver station. either this or @autonomous
public class HelloWorld extends OpMode { //part of the OpMode. HelloWorld takes all the characteristcs of OpMode


    @Override //overrides init from opmode
    public void init() {
        telemetry.addData("Hello", "World"); //prints hello world on screen
    }

    @Override
    public void loop(){
        //empty. allowed because we just need it for the requirement
    }
}
