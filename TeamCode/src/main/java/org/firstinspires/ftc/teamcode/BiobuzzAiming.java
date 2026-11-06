
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
public class BiobuzzAiming extends OpMode {
    MecanumDrive drive = new MecanumDrive(); //call class
    double forward, strafe, rotate;
    private Limelight3A limelight;
    private IMU imu;

    GoBildaPinpointDriver pinpoint;
    double error;
    double derivative, integral;
    double kp,kd,ki,ks,kv;
    double goalx,goaly;
    double goalAngle;
    double angleError;
    private ElapsedTime deltaTime = new ElapsedTime();





    @Override
    public void init(){

        //init turret
        //turretServo = hardwareMap.get(CRServo.class,"turretServo");

        //init drive/imu
        drive.init(hardwareMap);


        //init motors


        //init limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);

        imu = hardwareMap.get(IMU.class,"imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        // Get a reference to the sensor
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Configure the sensor
        configurePinpoint();

        // Set the location of the robot - this should be the place you are starting the robot from
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

    }

    @Override
    public void start(){
        limelight.start(); //if theres delay then put it into init but it drains battery
    }

    @Override
    public void loop(){



        if(gamepad1.y){
            // You could use readings from April Tags here to give a new known position to the pinpoint
            //pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
            imu.resetYaw();
            pinpoint.resetPosAndIMU();
        }
        pinpoint.update();
        telemetry.update();
        Pose2D pose2D = pinpoint.getPosition();

        forward = gamepad1.left_stick_y;
        strafe = -gamepad1.left_stick_x;
        rotate = -gamepad1.right_stick_x;

        drive.driveFieldRelative(forward, strafe, rotate,pose2D.getHeading(AngleUnit.RADIANS));

        telemetry.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
        telemetry.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
        telemetry.addData("Heading angle (DEGREES)",pose2D.getHeading(AngleUnit.DEGREES));
        telemetry.addData("xdir",pinpoint.getEncoderX());
        telemetry.addData("yButton", gamepad1.y);


        LLResult llResult = limelight.getLatestResult();

        telemetry.addData("isValid",llResult.isValid());

        telemetry.addData("Tag Count", llResult.getFiducialResults().size());



       if (llResult != null && llResult.isValid() && gamepad1.a) {
           telemetry.addData("tx", llResult.getTx());
           telemetry.addData("ty", llResult.getTy());
           telemetry.addData("ta", llResult.getTa());

           error = -llResult.getTx();
       } else if (gamepad1.a){
           //target: 64.5 in forward, 0 in sideways (start back to wall facing the target
           goalx = 64.5;
           goaly = 0;

           goalAngle = Math.toDegrees(Math.atan2(goaly - pose2D.getY(DistanceUnit.INCH), goalx - pose2D.getX(DistanceUnit.INCH)));
           telemetry.addData("Goal Angle: ", goalAngle);

           angleError = goalAngle - pinpoint.getHeading(AngleUnit.DEGREES);


           if (Math.abs(angleError)>=180){
               angleError = -1 * (angleError % 180);
           }

           telemetry.addData("Angle Error", angleError);


           kp = 0.05;




           rotate = Math.min(1.0, kp*angleError);
       }

       drive.driveFieldRelative(forward, strafe, rotate,pose2D.getHeading(AngleUnit.RADIANS));

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
        pinpoint.setOffsets(-30.0, -170, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
         * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
         * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
         * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
         * number of ticks per unit of your odometry pod.  For example:
         *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
         */
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
         * Set the direction that each of the two odometry pods count. The X (forward) pod should
         * increase when you move the robot forward. And the Y (strafe) pod should increase when
         * you move the robot to the left.
         */
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
        GoBildaPinpointDriver.EncoderDirection.REVERSED);

        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        pinpoint.resetPosAndIMU();
    }
}
