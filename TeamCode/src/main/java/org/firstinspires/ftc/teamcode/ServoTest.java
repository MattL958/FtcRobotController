package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTest extends OpMode {
    private Servo turretServo;
    private double angle;

    @Override
    public void init() {
        turretServo = hardwareMap.get(Servo.class,"turret_servo");
    }

    @Override
    public void loop() {
        angle = abs(gamepad1.left_stick_x);
        turretServo.setPosition(angle);
    }
}
