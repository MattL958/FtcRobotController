package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "mattapriltag2025 (Blocks to Java)")
public class mattapriltag2025 extends LinearOpMode {

    private DcMotor front_left_drive;
    private DcMotor front_right_drive;
    private DcMotor back_left_drive;
    private DcMotor back_right_drive;

    double Kp;
    double Kd;
    double Ki;
    AprilTagProcessor myAprilTagProcessor;
    long lastTime;
    boolean USE_WEBCAM;
    VisionPortal myVisionPortal;
    double Yaw;

    /**
     * This OpMode illustrates the basics of AprilTag recognition and pose estimation.
     *
     * For an introduction to AprilTags, see the FTC-DOCS link below:
     * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
     *
     * In this sample, any visible tag ID will be detected and
     * displayed, but only tags that are included in the default
     * "TagLibrary" will have their position and orientation information displayed. This default TagLibrary contains
     * the current Season's AprilTags and a small set of "test Tags" in the high number range.
     *
     * When an AprilTag in the TagLibrary is detected, the SDK provides
     * location and orientation of the tag, relative to the camera.
     * This information is provided in the "ftcPose" member of the returned
     * "detection", and is explained in the ftc-docs page linked below.
     * https://ftc-docs.firstinspires.org/apriltag-detection-values
     */
    @Override
    public void runOpMode() {
        float Axial;
        float Lateral;
        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;
        double maxPower;

        front_left_drive = hardwareMap.get(DcMotor.class, "front_left_drive");
        front_right_drive = hardwareMap.get(DcMotor.class, "front_right_drive");
        back_left_drive = hardwareMap.get(DcMotor.class, "back_left_drive");
        back_right_drive = hardwareMap.get(DcMotor.class, "back_right_drive");

        Kp = 0.0025;
        Kd = 0.0015;
        Ki = 0.0001;
        // Get the current time in milliseconds. The value returned represents
        // the number of milliseconds since midnight, January 1, 1970 UTC.
        lastTime = System.currentTimeMillis();
        USE_WEBCAM = true;
        // Initialize AprilTag before waitForStart.
        initAprilTag();
        // Wait for the match to begin.
        front_left_drive.setDirection(DcMotor.Direction.REVERSE);
        front_right_drive.setDirection(DcMotor.Direction.FORWARD);
        back_left_drive.setDirection(DcMotor.Direction.REVERSE);
        back_right_drive.setDirection(DcMotor.Direction.FORWARD);
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                if (gamepad1.dpad_down) {
                    // Temporarily stop the streaming session. This can save CPU
                    // resources, with the ability to resume quickly when needed.
                    myVisionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    // Resume the streaming session if previously stopped.
                    myVisionPortal.resumeStreaming();
                }
                // Share the CPU.
                sleep(20);
                Axial = -gamepad1.left_stick_y;
                Lateral = gamepad1.left_stick_x;
                Yaw = gamepad1.right_stick_x;
                // Push telemetry to the Driver Station.
                telemetry.update();
                telemetryAprilTag();
                telemetry.addData("Yaw", Yaw);
                frontLeftPower = Axial + Lateral + Yaw;
                frontRightPower = (Axial - Lateral) - Yaw;
                backLeftPower = (Axial - Lateral) + Yaw;
                backRightPower = (Axial + Lateral) - Yaw;
                maxPower = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(frontLeftPower), Math.abs(frontRightPower), Math.abs(backLeftPower), Math.abs(backRightPower)));
                if (maxPower > 1) {
                    frontLeftPower = frontLeftPower / maxPower;
                    frontRightPower = frontRightPower / maxPower;
                    backLeftPower = backLeftPower / maxPower;
                    backRightPower = backRightPower / maxPower;
                }
                front_left_drive.setPower(frontLeftPower);
                front_right_drive.setPower(frontRightPower);
                back_left_drive.setPower(backLeftPower);
                back_right_drive.setPower(backRightPower);
            }
        }
    }

    /**
     * Initialize AprilTag Detection.
     */
    private void initAprilTag() {
        AprilTagProcessor.Builder myAprilTagProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;

        // First, create an AprilTagProcessor.Builder.
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        // Create an AprilTagProcessor by calling build.
        myAprilTagProcessor = myAprilTagProcessorBuilder.build();
        // Next, create a VisionPortal.Builder and set attributes related to the camera.
        myVisionPortalBuilder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            // Use a webcam.
            myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            // Use the device's back camera.
            myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
        }
        // Add myAprilTagProcessor to the VisionPortal.Builder.
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
        // Create a VisionPortal by calling build.
        myVisionPortal = myVisionPortalBuilder.build();
    }

    /**
     * Display info (using telemetry) for a recognized AprilTag.
     */
    private void telemetryAprilTag() {
        List<AprilTagDetection> myAprilTagDetections;
        AprilTagDetection myAprilTagDetection;
        int ButtonAIsHeld;
        long deltaTime;
        double Present_Value;
        int Setpoint;
        double Error2;
        double Derivative;
        int Integral;
        double lastError;

        // Get a list of AprilTag detections.
        myAprilTagDetections = myAprilTagProcessor.getDetections();
        telemetry.addData("# AprilTags Detected", JavaUtil.listLength(myAprilTagDetections));
        // Iterate through list and call a function to display info for each recognized AprilTag.
        for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
            myAprilTagDetection = myAprilTagDetection_item;
            // Display info about the detection.
            telemetry.addLine("");
            if (myAprilTagDetection.metadata != null) {
                telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") " + myAprilTagDetection.metadata.name);
                telemetry.addLine("XYZ " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.x, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.y, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.z, 6, 1) + "  (inch)");
                telemetry.addLine("PRY " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.pitch, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.roll, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.yaw, 6, 1) + "  (deg)");
                telemetry.addLine("RBE " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.range, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.bearing, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.elevation, 6, 1) + "  (inch, deg, deg)");
                if (gamepad1.a_was_pressed()) {
                    ButtonAIsHeld = 1;
                } else if (gamepad1.a_was_released()) {
                    ButtonAIsHeld = 0;
                }
                telemetry.addData("A", ButtonAIsHeld);
                if (myAprilTagDetection.id == 24 && ButtonAIsHeld == 1) {
                    // Get the current time in milliseconds. The value returned represents
                    // the number of milliseconds since midnight, January 1, 1970 UTC.
                    deltaTime = (System.currentTimeMillis() - lastTime) * 1000;
                    if (deltaTime == 0) {
                        deltaTime = 70;
                    }
                    Present_Value = myAprilTagDetection.center.x;
                    Setpoint = 320;
                    Error2 = 320 - myAprilTagDetection.center.x;
                    Derivative = (Error2 - lastError) / deltaTime;
                    Integral += Error2 * deltaTime;
                    if (Integral > 100) {
                        Integral = 100;
                    } else if (Integral < -100) {
                        Integral = -100;
                    }
                    Yaw = -(Error2 * Kp + Derivative * Kd + Integral * Ki);
                    if (Yaw < 0.05 && Yaw > -0.05) {
                        Yaw = 0;
                    }
                    telemetry.addData("Present Value", Present_Value);
                    telemetry.addData("Error", Error2);
                    telemetry.addData("Yaw", Yaw);
                    telemetry.addData("deltaTime", deltaTime);
                    lastError = Error2;
                    // Get the current time in milliseconds. The value returned represents
                    // the number of milliseconds since midnight, January 1, 1970 UTC.
                    lastTime = System.currentTimeMillis();
                }
            } else {
                telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") Unknown");
                telemetry.addLine("Center " + JavaUtil.formatNumber(myAprilTagDetection.center.x, 6, 0) + "" + JavaUtil.formatNumber(myAprilTagDetection.center.y, 6, 0) + " (pixels)");
            }
        }
    }
}