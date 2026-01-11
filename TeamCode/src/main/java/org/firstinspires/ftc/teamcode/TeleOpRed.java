package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@TeleOp
public class TeleOpRed extends OpMode {
    MecanumDrive drive = new MecanumDrive(); //call class
    double forward, strafe, rotate;
    private DcMotor intake;
    private IMU imu;
    private Limelight3A limelight;
    private CRServo turretServo;

    double error;

    double turretPower;
    double kp,kd,ki;

    TelemetryPacket packet = new TelemetryPacket();

    FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void init(){

        //init turret
        turretServo = hardwareMap.get(CRServo.class,"turretServo");

        //init drive/imu
        drive.init(hardwareMap);
        imu = hardwareMap.get(IMU.class, "imu");


        //init motors
        intake = hardwareMap.get(DcMotorEx.class,"intake");

        intake.setDirection(DcMotor.Direction.FORWARD);

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //init limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8); //whichever pipeline is the apriltag one

        imu = hardwareMap.get(IMU.class,"imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

    }

    @Override
    public void start(){
        limelight.start(); //if theres delay then put it into init but it drains battery
    }

    @Override
    public void loop(){

        //drive system
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        telemetry.addData("Yaw: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        drive.driveFieldRelative(forward, strafe, rotate);


        //intake
        double right_trigger = gamepad1.right_trigger;
        intake.setPower(right_trigger);

        telemetry.addData("Right Trigger", right_trigger);


        //apriltag recognition/telemetry
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();

        telemetry.addData("Yaw",orientation.getYaw());

        telemetry.addData("isValid",llResult.isValid());

        telemetry.addData("Tag Count", llResult.getFiducialResults().size());


        if (llResult != null && llResult.isValid()){
            Pose3D botPose = llResult.getBotpose_MT2();
            telemetry.addData("tx", llResult.getTx());
            telemetry.addData("ty",llResult.getTy());
            telemetry.addData("ta", llResult.getTa());
            telemetry.addData("BotPose", botPose.toString());
        } else {
            telemetry.addData("No Tag Found","");
        }



        //turret PID
        kp=0.1;

        error = llResult.getTx();
        turretPower = kp*error;
        //remap
        turretPower = turretPower/(25*kp); //max error should be 25 degrees, so this should normalize. results in a value between -1 and 1

        if(Math.abs(turretPower)>1){
            turretPower = Math.abs(turretPower)/turretPower;
        }

        turretServo.setPower(turretPower);

        packet.put("error",error);
    }
}
