package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

public class FourFarRed {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    //see if push in is needed
    private final Pose startPose = new Pose(87.9, 7.9, Math.toRadians(90));
    private final Pose score1Pose = new Pose(87.9, 12.4, Math.toRadians(72));
    private final Pose pickup1Pose = new Pose(126.5, 36.07, Math.toRadians(0));
    private final Pose score2Pose = new Pose(87.9, 12.4, Math.toRadians(72));
    private final Pose pickup2Pose = new Pose(126.5, 59.74, Math.toRadians(0));
    private final Pose score3Pose = new Pose(81.9, 75.6, Math.toRadians(46));
    private final Pose pickup3Pose = new Pose(126.5, 84.4, Math.toRadians(0));
    private final Pose score4Pose = new Pose(101.6, 96.8, Math.toRadians(47));
    private final Pose parkPose = new Pose(108.8, 90.6, Math.toRadians(128));


    private Path scorePreload;
    public PathChain pickup1, score2, pickup2, score3, pickup3, score4, park;

    public void buildPaths() {

        scorePreload = new Path(new BezierLine(startPose, score1Pose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), score1Pose.getHeading());


        pickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(score1Pose, pickup1Pose, new Pose(101.7, 34.9)))
                .setLinearHeadingInterpolation(score1Pose.getHeading(), pickup1Pose.getHeading())
                .setBrakingStart(4)
                .setBrakingStrength(0.04)
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, score2Pose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), score2Pose.getHeading())
                .setBrakingStart(4)
                .setBrakingStrength(0.04)
                .build();

        pickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(score2Pose, pickup2Pose, new Pose(96.4, 63.1)))
                .setLinearHeadingInterpolation(score2Pose.getHeading(), pickup2Pose.getHeading())
                .setBrakingStart(4)
                .setBrakingStrength(0.04)
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, score3Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), score3Pose.getHeading())
                .setBrakingStart(4)
                .setBrakingStrength(0.04)
                .build();


        pickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(score3Pose, pickup3Pose, new Pose(92.3, 82.1)))
                .setLinearHeadingInterpolation(score3Pose.getHeading(), pickup3Pose.getHeading())
                .setBrakingStart(4)
                .setBrakingStrength(0.04)
                .build();

        score4 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, score4Pose))
                .setConstantHeadingInterpolation(score4Pose.getHeading())
                .setBrakingStart(4)
                .setBrakingStrength(0.04)
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(score4Pose, parkPose))
                .setConstantHeadingInterpolation(score4Pose.getHeading())
                .build();

    }
}

