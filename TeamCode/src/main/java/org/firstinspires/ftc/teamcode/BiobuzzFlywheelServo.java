package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class BiobuzzFlywheelServo extends OpMode {
    private Servo flywheel_servo;
    double servoPos;
    @Override
    public void init(){
        flywheel_servo = hardwareMap.get(Servo.class, "flywheel_servo");

        servoPos = 0;
    }

    @Override
    public void loop(){
        if(gamepad1.aWasPressed()){
            servoPos=0.1;
        }
        if (gamepad1.bWasPressed()){
            servoPos=0.6;
        }
        if(servoPos > 1){
            servoPos = 1;
        } else if (servoPos < 0){
            servoPos = 0;
        }
        telemetry.addData("pos",servoPos);
        flywheel_servo.setPosition(servoPos);
    }
}
