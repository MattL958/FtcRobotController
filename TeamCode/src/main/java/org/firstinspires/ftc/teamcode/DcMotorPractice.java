package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.mechanisms.Motors;
@TeleOp
public class DcMotorPractice extends OpMode {
    //Motors motors = new Motors();
    private DcMotor front_left_drive; //name this
    private double ticksPerRev;

    @Override
    public void init() {
        front_left_drive = hardwareMap.get(DcMotor.class,"front_left_drive");
        front_left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        ticksPerRev = front_left_drive.getMotorType().getTicksPerRev();
    }

    @Override
    public void loop(){
        front_left_drive.setPower(0.5);
        telemetry.addData("Revolutions",front_left_drive.getCurrentPosition() / ticksPerRev);
    }
}
