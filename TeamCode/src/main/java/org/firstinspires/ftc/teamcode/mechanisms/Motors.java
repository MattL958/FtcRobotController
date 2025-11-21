package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Motors  {
    private DcMotor front_right_drive; //name this

    public void init(HardwareMap hwMap){
        front_right_drive = hwMap.get(DcMotor.class,"front_right_drive");
        front_right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right_drive.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    public void setMotorSpeed(double speed){
        //accepts values from -1.0 to 1.0
        front_right_drive.setPower(speed);
    }
}
