package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class BiobuzzFlywheel extends OpMode {
    private Servo flywheel_servo;
    double servoPos;

    private DcMotorEx flywheel;
    @Override
    public void init(){
        flywheel_servo = hardwareMap.get(Servo.class, "flywheel_servo");

        servoPos = 0;

        flywheel = hardwareMap.get(DcMotorEx.class,"front_left_drive");
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorEx.Direction.REVERSE);


    }

    @Override
    public void loop(){
        if(gamepad1.aWasPressed()){
            servoPos=0.1;
        }
        if (gamepad1.bWasPressed()){
            servoPos=0.6;
        }


        flywheel.setPower(gamepad1.left_stick_y);
        flywheel_servo.setPosition(servoPos);


        telemetry.addData("flywheelvel",flywheel.getVelocity());
        telemetry.addData("pos",servoPos);


    }
}
