
package org.firstinspires.ftc.teamcode;


import static java.lang.Math.abs;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;


@TeleOp
public class TeleOpRed extends OpMode {
    MecanumDrive drive = new MecanumDrive(); //call class
    double forward, strafe, rotate;
    private DcMotor intake;
    private DcMotorEx shooting;
    private DcMotor transfer1;
    private DcMotor transfer2;
    private Servo transfer_servo;
    private IMU imu;
    //private Limelight3A limelight;
    //private CRServo turretServo;
    //private DcMotor left_transfer;
    //private DcMotor right_transfer;
    //private Servo transfer_servo;
    //GoBildaPinpointDriver pinpoint;

    double error;
    double last_error = 0.0;
    double derivative, integral;

    double turretPower;
    public static double kp,kd,ki,ks,kv; //public static shows up in dashboard config
    double shooterVelocity,targetVelocity;
    private ElapsedTime deltaTime = new ElapsedTime();
    double[] errorArr = new double[5];
    int count = 0;
    double sum;

    //FtcDashboard dashboard = FtcDashboard.getInstance();



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

        transfer_servo = hardwareMap.get(Servo.class,"transfer_servo");
        //left_transfer = hardwareMap.get(DcMotor.class,"left_transfer");

        //left_transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        //left_transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //145.1 ticks/rev acc to website

        //right_transfer = hardwareMap.get(DcMotor.class, "right_transfer");

        //right_transfer.setDirection(DcMotor.Direction.FORWARD);

        //right_transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //transfer_servo = hardwareMap.get(Servo.class,"transfer_servo");



        //init limelight
        //limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //limelight.pipelineSwitch(8); //8 = red apriltag (24)

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

        targetVelocity = 1700;
    }

    @Override
    public void start(){
        //limelight.start(); //if theres delay then put it into init but it drains battery
    }

    @Override
    public void loop(){



        if(gamepad1.y){
            // You could use readings from April Tags here to give a new known position to the pinpoint
            //pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
            imu.resetYaw();
        }
        //pinpoint.update();
        //Pose2D pose2D = pinpoint.getPosition();

        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = -gamepad1.right_stick_x;

        drive.driveFieldRelative(forward, strafe, rotate);

        telemetry.addData("Yaw: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        //drive.driveFieldRelative(forward, strafe, rotate, pose2D.getHeading(AngleUnit.RADIANS));

        //telemetry.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
        //telemetry.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
        //telemetry.addData("Heading angle (DEGREES)",pose2D.getHeading(AngleUnit.DEGREES));

        //drive.driveFieldRelative(forward, strafe, rotate);


        //intake
        double right_trigger = gamepad1.right_trigger;
        intake.setPower(right_trigger);
        transfer2.setPower(right_trigger);
        if(gamepad1.x){
            intake.setPower(-1.0);
            transfer2.setPower(-1.0);
            transfer1.setPower(-1.0);
        }
        if(gamepad1.a && abs(error) <= 20){
            transfer1.setPower(1.0);
            intake.setPower(1.0);
            transfer2.setPower(1.0);
        } else {
            transfer1.setPower(0.0);
        }

        telemetry.addData("Right Trigger", right_trigger);


        //apriltag recognition/telemetry
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        //limelight.updateRobotOrientation(pose2D.getHeading(AngleUnit.DEGREES));
        //LLResult llResult = limelight.getLatestResult();

        //telemetry.addData("isValid",llResult.isValid());

        //telemetry.addData("Tag Count", llResult.getFiducialResults().size());



/*
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
        turretServo.setPower(0);



        boolean aButton = gamepad1.a;
        boolean bButton = gamepad1.b;

        if(gamepad1.left_trigger != 0.0){
            shooting.setPower(gamepad1.left_trigger);
        } else if (aButton){
            shooting.setPower(-1);
        } else {
            shooting.setPower(0.0);
        }

*/
        kp = 1; //0.15
        ks = 0.05; //0.05
        kv = 0.000398; //0.000372485


        //target vel prolly 2200 far 2000 near
        if(gamepad1.dpadDownWasPressed()){
            targetVelocity -= 20;
        } else if (gamepad1.dpadUpWasPressed()){
            targetVelocity += 20;
        }

        telemetry.addData("targetVelocity",targetVelocity);

        shooterVelocity = shooting.getVelocity();
        error = targetVelocity - shooterVelocity;
        double feedback = kp*error;
        double feedforward = ks + kv*targetVelocity;
        shooting.setPower(feedback+feedforward);



/*
        if(gamepad1.right_trigger != 0){
            transfer_servo.setPosition(0);
        } else {
            transfer_servo.setPosition(0.15);
        }
        */
 ;;


        //telemetry.addData("A Button",aButton);
        //telemetry.addData("B Button", bButton);


        telemetry.addData("Flywheel",shooting.getVelocity());
        telemetry.addData("Error",error);



        /*
        TelemetryPacket packet = new TelemetryPacket(); //create a new packet each loop
        packet.put("error",error);


        dashboard.sendTelemetryPacket(packet);
        */


    }

    public void configurePinpoint(){
        /*
         *  Set the odometry pod positions relative to the point that you want the position to be measured from.
         *
         *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
         *  Left of the center is a positive number, right of center is a negative number.
         *
         *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
         *  Forward of center is a positive number, backwards is a negative number.
         */
        //pinpoint.setOffsets(12.5, -170, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
         * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
         * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
         * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
         * number of ticks per unit of your odometry pod.  For example:
         *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
         */
        //pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
         * Set the direction that each of the two odometry pods count. The X (forward) pod should
         * increase when you move the robot forward. And the Y (strafe) pod should increase when
         * you move the robot to the left.
         */
        //pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                //GoBildaPinpointDriver.EncoderDirection.REVERSED);

        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        //pinpoint.resetPosAndIMU();
    }
}
