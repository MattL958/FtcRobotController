package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrive {
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor; //class DcMotor, variables there
    private IMU imu; //class IMU, variable named imu

    public void init(HardwareMap hwMap){
        frontLeftMotor = hwMap.get(DcMotor.class,"front_left_drive");
        backLeftMotor = hwMap.get(DcMotor.class,"back_left_drive");
        frontRightMotor = hwMap.get(DcMotor.class,"front_right_drive");
        backRightMotor = hwMap.get(DcMotor.class,"back_right_drive");

        //typically in a mecanum drive one side is reversed, traditionally its the left

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hwMap.get(IMU.class,"imu"); //orientation is based on control hub and usb direction. e.g. logo top usb front = facing forward

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot( //defines paramaters for IMU. Logo direction and USB
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);



        imu.initialize(new IMU.Parameters(RevOrientation));

        imu.resetYaw();

    }

    public void drive(double forward, double strafe, double rotate){
        //need "fancy math" according to brogan pratt
        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        double maxPower = 1.0; //clamp all values to 1.0
        double maxSpeed = 1.0; //if we need to slow it down, i.e. if we have someone incompetent driving

        maxPower = Math.max(maxPower,Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower,Math.abs(backLeftPower));
        maxPower = Math.max(maxPower,Math.abs(frontRightPower));
        maxPower = Math.max(maxPower,Math.abs(backRightPower));
        //finds the highest power among all four

        frontLeftMotor.setPower(maxSpeed * (frontLeftPower/maxPower));
        backLeftMotor.setPower(maxSpeed * (backLeftPower/maxPower));
        frontRightMotor.setPower(maxSpeed * (frontRightPower/maxPower));
        backRightMotor.setPower(maxSpeed * (backRightPower/maxPower));

        //up to here, this is robot oriented driving
    }



    public void driveFieldRelative(double forward, double strafe, double rotate){
        //converting from cartesian to polar
        double theta = Math.atan2(forward,strafe);
        double r = Math.hypot(strafe,forward);

        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        //turn polar back to cartesian
        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        this.drive(newForward,newStrafe,rotate);

    }
}
