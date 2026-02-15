package org.firstinspires.ftc.teamcode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class farRed extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private DcMotor intake;
    private DcMotorEx shooting;
    private DcMotor transfer1;
    private DcMotor transfer2;
    private Timer opmodeTimer, pathTimer;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        intake = hardwareMap.get(DcMotor.class,"intake");

        intake.setDirection(DcMotor.Direction.FORWARD);

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooting = hardwareMap.get(DcMotorEx.class, "shooting");

        shooting.setDirection(DcMotorEx.Direction.FORWARD);

        shooting.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        transfer1 = hardwareMap.get(DcMotor.class,"transfer1");
        transfer2 = hardwareMap.get(DcMotor.class,"transfer2");

        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        pathTimer = new Timer();
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        //pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start(){
        opmodeTimer.resetTimer();
        setPathState(0);
    }


    public static class Paths {
        public PathChain start;
        public PathChain intake1;
        public PathChain return1;
        public PathChain move2;
        public PathChain intake2;
        public PathChain return2;

        public Paths(Follower follower) {
            start = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.000, 8.000),

                                    new Pose(99.175, 35.002)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))

                    .build();

            intake1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(99.175, 35.002),

                                    new Pose(125.485, 35.444)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            return1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125.485, 35.444),

                                    new Pose(79.069, 15.627)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(230))

                    .build();

            move2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(79.069, 15.627),

                                    new Pose(107.250, 54.017)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(25))

                    .build();

            intake2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(107.250, 54.017),

                                    new Pose(128.499, 67.414)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(25))

                    .build();

            return2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(128.499, 67.414),

                                    new Pose(80.007, 14.546)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(230))

                    .build();
        }
    }


    public void autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        switch (pathState) {
            case 0:
                follower.followPath(paths.start);
                setPathState(1);
                break;
            case 1:
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(paths.intake1,true);
                    intake.setPower(1.0);
                    transfer2.setPower(1.0);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(paths.return1,true);
                    if(pathTimer.getElapsedTimeSeconds() < 4) {
                        setPathState(3);
                    }
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(paths.move2,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(paths.intake2,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(paths.return2,true);
                    if(pathTimer.getElapsedTimeSeconds() < 4){
                        setPathState(-1);
                    }
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
    