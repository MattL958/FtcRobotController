package org.firstinspires.ftc.teamcode;
import android.media.audiofx.Visualizer;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Disabled
@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class farBlue extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
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
                                    new Pose(56.000, 8.000),

                                    new Pose(44.769, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                    .build();

            intake1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(44.769, 36.000),

                                    new Pose(15.177, 36.192)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            return1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.177, 36.192),

                                    new Pose(66.841, 17.624)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(300))

                    .build();

            move2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(66.841, 17.624),

                                    new Pose(35.873, 53.768)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(300), Math.toRadians(135))

                    .build();

            intake2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(35.873, 53.768),

                                    new Pose(13.948, 67.414)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(135))

                    .build();

            return2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(13.948, 67.414),

                                    new Pose(67.029, 17.790)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(300))

                    .build();
        }
    }


    public void autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
    }
}









