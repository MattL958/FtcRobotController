package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class ServoTest2 extends OpMode {

    private CRServo turretServo;
    private double power;

    @Override
    public void init() {
        turretServo = hardwareMap.get(CRServo.class,"turretServo");

    }

    @Override
    public void loop() {
        power = gamepad1.left_stick_x;
        turretServo.setPower(power);
    }
}
