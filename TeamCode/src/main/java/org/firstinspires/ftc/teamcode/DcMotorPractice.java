package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.mechanisms.Motors;
@TeleOp
public class DcMotorPractice extends OpMode {
    //Motors motors = new Motors();
    private DcMotorEx front_left_drive, front_right_drive; //name this
    private double ticksPerRev;

    @Override
    public void init() {
        front_left_drive = hardwareMap.get(DcMotorEx.class,"front_left_drive");
        front_left_drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        front_left_drive.setDirection(DcMotorEx.Direction.REVERSE);


        front_right_drive = hardwareMap.get(DcMotorEx.class,"front_right_drive");
        front_right_drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        front_right_drive.setDirection(DcMotorEx.Direction.REVERSE);
    }

    @Override
    public void loop(){
        front_left_drive.setPower(gamepad1.left_stick_y);
        front_right_drive.setPower(gamepad1.right_stick_y);
        telemetry.addData("LeftVel",front_left_drive.getVelocity());
        telemetry.addData("RightVel",front_right_drive.getVelocity());
    }
}
