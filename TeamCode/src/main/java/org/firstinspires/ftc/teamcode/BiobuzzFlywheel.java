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
    double flywheelPower;
    double targetVelocity;
    double ks,kv,kp;
    double flywheelError;
    @Override
    public void init(){
        flywheel_servo = hardwareMap.get(Servo.class, "flywheel_servo");

        servoPos = 0;

        flywheel = hardwareMap.get(DcMotorEx.class,"front_left_drive");
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorEx.Direction.FORWARD);

        flywheelPower = 0.8;

        targetVelocity = 1500;

    }

    @Override
    public void loop(){
        if(gamepad1.aWasPressed()){ //change these controls next bc ts bs
            servoPos=0.1;
        }
        if (gamepad1.bWasPressed()){
            servoPos=0.6;
        }

        if(gamepad1.dpadUpWasPressed()){
            targetVelocity+=20;
        }
        if (gamepad1.dpadDownWasPressed()){
            targetVelocity-=20;
        }

        //power 0.8 -> 1900 vel
        ks = 0.00075;
        kv = 0.000425;
        kp = 0.001;
        flywheelError = targetVelocity - flywheel.getVelocity();
        flywheel.setPower(ks + kv*targetVelocity + kp*flywheelError);
        flywheel_servo.setPosition(servoPos);

        telemetry.addData("flywheelpower",flywheelPower);
        telemetry.addData("flywheelvel",flywheel.getVelocity());
        telemetry.addData("targetvel",targetVelocity);
        telemetry.addData("pos",servoPos);


    }
}
