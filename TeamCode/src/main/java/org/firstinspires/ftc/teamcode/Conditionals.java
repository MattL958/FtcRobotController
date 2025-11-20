package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Conditionals extends OpMode {
    @Override
    public void init() {

    }

    double squareInputWithSigns(double input) {
        double output = input * input;
        if (input<0){
            output*=-1;
        }
        return output;
    }
    @Override
    public void loop() {
        boolean aButton = gamepad1.a;
        if (aButton) {
            telemetry.addData("A Button","Pressed");
        }
        else {
            telemetry.addData("A Button","Not Pressed");
        }
        telemetry.addData("A Button State",aButton);

        telemetry.addData("left joystick y",gamepad1.left_stick_y);
        double yAxis = squareInputWithSigns(gamepad1.left_stick_y);
        telemetry.addData("squared left joystick y", yAxis);
    }
}
