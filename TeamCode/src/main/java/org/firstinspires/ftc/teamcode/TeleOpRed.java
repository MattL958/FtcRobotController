package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;


@TeleOp
public class TeleOpRed extends OpMode {
    MecanumDrive drive = new MecanumDrive(); //call class
    double forward, strafe, rotate;
    private DcMotor intake;
    private DcMotor shooting;
    private IMU imu;
    private Limelight3A limelight;
    private CRServo turretServo;
    private DcMotor left_transfer;
    private DcMotor right_transfer;

    double error;
    double last_error = 0.0;
    double derivative, integral;

    double turretPower;
    public static double kp,kd,ki; //public static shows up in dashboard config
    private ElapsedTime deltaTime = new ElapsedTime();
    double[] errorArr = new double[5];
    int count = 0;
    double sum;

    FtcDashboard dashboard = FtcDashboard.getInstance();



    @Override
    public void init(){

        //init turret
        turretServo = hardwareMap.get(CRServo.class,"turretServo");

        //init drive/imu
        drive.init(hardwareMap);


        //init motors
        intake = hardwareMap.get(DcMotor.class,"intake");

        intake.setDirection(DcMotor.Direction.FORWARD);

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooting = hardwareMap.get(DcMotorEx.class, "shooting");

        shooting.setDirection(DcMotor.Direction.FORWARD);

        shooting.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left_transfer = hardwareMap.get(DcMotor.class,"left_transfer");

        left_transfer.setDirection(DcMotorSimple.Direction.FORWARD);

        left_transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        right_transfer = hardwareMap.get(DcMotor.class, "right_transfer");

        right_transfer.setDirection(DcMotor.Direction.FORWARD);

        right_transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

            error = -llResult.getTx();



            errorArr[count] = error;
            count +=1;
            if(count > 4){
                count = 0;
            }



            sum = 0;
            for(int i = 0; i<errorArr.length;i++){
                sum += errorArr[i];
            }

            error = sum/errorArr.length;
        } else {
            telemetry.addData("No Tag Found","");
            error = 0.0;
            integral = 0;
        }



        //turret PID


        kp=0.01;
        kd=0.0008;
        ki=0.000;

        if(last_error==0.0){
            last_error=error;
        }

        telemetry.addData("error",error);

        derivative = (error-last_error)/Math.max(deltaTime.seconds(),0.01);
        if (Math.abs(error)<1){
            derivative=0.0;
        }
        if (Math.abs(derivative)>50){
            derivative=50;
        }
        integral += error * deltaTime.seconds();

        turretPower = kp*error + kd*derivative + ki*integral;
        telemetry.addData("dt",deltaTime.seconds());
        telemetry.addData("error-last_error = ",error-last_error);
        telemetry.addData("p",kp*error);
        telemetry.addData("d",kd*derivative);
        telemetry.addData("i",ki*integral);

        deltaTime.reset();
        last_error=error;

        //clamp
        if(turretPower>1.0){
            turretPower = 1.0;
        } else if (turretPower<-1.0) {
            turretPower = -1.0;
        }

        if (!(llResult != null && llResult.isValid())){
            turretPower=0;
        }

        //send power
        telemetry.addData("turretPower",turretPower);
        turretServo.setPower(turretPower);

        shooting.setPower(gamepad1.left_trigger);

        boolean aButton = gamepad1.a;
        boolean bButton = gamepad1.b;
        if(bButton){
            left_transfer.setPower(0.2);
            right_transfer.setPower(0.2);
        } else if(aButton){
            left_transfer.setPower(-0.2);
            right_transfer.setPower(-0.2);
        } else {
            left_transfer.setPower(0.0);
            right_transfer.setPower(0.0);
        }

        telemetry.addData("A Button",aButton);
        telemetry.addData("B Button", bButton);

        boolean yButton = gamepad1.y;
        if(yButton){
            imu.resetYaw();
        }


        /*
        TelemetryPacket packet = new TelemetryPacket(); //create a new packet each loop
        packet.put("error",error);


        dashboard.sendTelemetryPacket(packet);
        */
    }
}
