package org.firstinspires.ftc.teamcode.paths;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

public class FourBlueFar{

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    //see if push in is needed
    private final Pose startPose = new Pose(58, 7.9, Math.toRadians(90));
    private final Pose score1Pose = new Pose(57.77, 15.5, Math.toRadians(111));
    private final Pose pickup1Pose = new Pose(16.91, 36.07, Math.toRadians(180));
    private final Pose score2Pose = new Pose(57.77, 15.5, Math.toRadians(111));
    private final Pose pickup2Pose = new Pose(16.91, 59.74, Math.toRadians(180));
    private final Pose score3Pose = new Pose(60, 75.8, Math.toRadians(133));
    private final Pose pickup3Pose = new Pose(16.91, 84, Math.toRadians(180));
    private final Pose score4Pose = new Pose(38.3, 95.5, Math.toRadians(128));
    private final Pose parkPose = new Pose(33.5, 92.7, Math.toRadians(128));



    private Path scorePreload;
    public PathChain pickup1, score2, pickup2, score3, pickup3, score4, park;

    public void buildPaths() {

        scorePreload = new Path(new BezierLine(startPose, score1Pose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), score1Pose.getHeading());


        pickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(score1Pose, pickup1Pose, new Pose(45.93, 36.92)))
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
                .addPath(new BezierCurve(score2Pose, pickup2Pose, new Pose(46.5, 62.55)))
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
                .addPath(new BezierCurve(score3Pose, pickup3Pose, new Pose(47.1, 85.1)))
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
                .setBrakingStart(4)
                .setBrakingStrength(0.04)
                .build();

    }
}


