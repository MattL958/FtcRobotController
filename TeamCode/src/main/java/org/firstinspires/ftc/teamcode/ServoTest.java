package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTest extends OpMode {
    private Servo transfer_servo;
    private double angle;

    @Override
    public void init() {
        transfer_servo = hardwareMap.get(Servo.class,"transfer_servo");
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            transfer_servo.setPosition(0.25);
        } else {
            transfer_servo.setPosition(0);
        }
    }
}
