package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Autonomous(name = "Autonomous", group = "Autonomous")

public class Auto extends OpMode {
    MecanumDrive drive = new MecanumDrive(); //call class
    double forward, strafe, rotate;
    private DcMotor intake;
    private DcMotorEx shooting;
    private DcMotor transfer1;
    private DcMotor transfer2;
    private IMU imu;
    public static double kp,kd,ki,ks,kv; //public static shows up in dashboard config
    double shooterVelocity,targetVelocity,error;
    private ElapsedTime deltaTime = new ElapsedTime();
    @Override
    public void init(){

        //init turret
        //turretServo = hardwareMap.get(CRServo.class,"turretServo");

        //init drive/imu
        drive.init(hardwareMap);


        //init motors
        intake = hardwareMap.get(DcMotor.class,"intake");

        intake.setDirection(DcMotor.Direction.FORWARD);

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooting = hardwareMap.get(DcMotorEx.class, "shooting");

        shooting.setDirection(DcMotorEx.Direction.FORWARD);

        shooting.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        transfer1 = hardwareMap.get(DcMotor.class,"transfer1");
        transfer2 = hardwareMap.get(DcMotor.class,"transfer2");


        imu = hardwareMap.get(IMU.class,"imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        // Get a reference to the sensor
        //pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Configure the sensor
        //configurePinpoint();

        // Set the location of the robot - this should be the place you are starting the robot from
        //pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        targetVelocity = 1820;
    }


    @Override
    public void start(){
        deltaTime.reset();
    }
    @Override
    public void loop(){
        kp = 1; //0.15
        ks = 0.05; //0.05
        kv = 0.000398; //0.000372485




        telemetry.addData("targetVelocity",targetVelocity);
        telemetry.addData("Flywheel",shooterVelocity);

        if(deltaTime.seconds()<10){
            shooterVelocity = shooting.getVelocity();
            error = targetVelocity - shooterVelocity;
            double feedback = kp*error;
            double feedforward = ks + kv*targetVelocity;
            shooting.setPower(feedback+feedforward);
        }


        if(shooterVelocity >= 1800){

            transfer1.setPower(1.0);
            intake.setPower(1.0);
            transfer2.setPower(1.0);
        }

        if(deltaTime.seconds() > 10){
            transfer1.setPower(0.0);
            intake.setPower(0.0);
            transfer2.setPower(0.0);
        }

        if(deltaTime.seconds()>10 && deltaTime.seconds() < 11){
            drive.driveFieldRelative(1.0,0.0,0.0);
        } else {
            drive.driveFieldRelative(0.0,0.0,0.0);
        }
    }
}
