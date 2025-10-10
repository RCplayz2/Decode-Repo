package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.BezierPoint;

public class FourCycleBlue {
    private Follower follower;
    public static Pose startPose = new Pose(40, 135, Math.toRadians(90));
    public static Pose score1Pose = new Pose(44, 92, Math.toRadians(135));
    public static Pose pickup1Pose = new Pose(15.781, 83.695, Math.toRadians(180));
    // dont know if runIn1 is needed or if can just make a bezier curve that runs in a curve so that it works like this.
//    public static Pose runIn1 = new Pose(...);
    public static Pose score2Pose = new Pose(58.851, 83.977, Math.toRadians(135));
    public static Pose pickup2Pose = new Pose(15.499, 59.742, Math.toRadians(180));
//    public static Pose runIn2 = new Pose(...);
    public static Pose score3Pose = new Pose(58.896, 76.368, Math.toRadians(130));
    public static Pose pickup3Pose = new Pose(15.217, 36.070, Math.toRadians(180));
//    public static Pose runIn3 = new Pose(...);
    public static Pose score4Pose = new Pose(62.278, 72.132, Math.toRadians(132));
    public static Pose parkPose = new Pose(56.2, 69, Math.toRadians(125));
// add pickup from the human player zone if time is remaining after testing

    private Path scorePreload;
    public PathChain pickup1, score2, pickup2, score3, pickup3, score4, park;

    //see if push in is needed
    public void buildPaths() {

        scorePreload = new Path(new BezierLine(startPose, score1Pose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), score1Pose.getHeading());



        pickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(score1Pose, pickup1Pose, new Pose(58.615, 82.286)))
                .setLinearHeadingInterpolation(score1Pose.getHeading(), pickup1Pose.getHeading())
                .setBrakingStart(4)
                .setBrakingStrength(0.04)
                .build();
        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        score2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, score2Pose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), score2Pose.getHeading())
                .setBrakingStart(4)
                .setBrakingStrength(0.04)
                .build();
        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        pickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(score2Pose, pickup2Pose, new Pose(62.560, 55.515)))
                .setLinearHeadingInterpolation(score2Pose.getHeading(), pickup2Pose.getHeading())
                .setBrakingStart(4)
                .setBrakingStrength(0.04)
                .build();
        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        score3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, score3Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), score3Pose.getHeading())
                .setBrakingStart(4)
                .setBrakingStrength(0.04)
                .build();
        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        pickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(score3Pose, pickup3Pose, new Pose(60.023, 24.517)))
                .setLinearHeadingInterpolation(score3Pose.getHeading(), pickup3Pose.getHeading())
                .setBrakingStart(4)
                .setBrakingStrength(0.04)
                .build();
        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        score4 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, score4Pose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), score4Pose.getHeading())
                .setBrakingStart(4)
                .setBrakingStrength(0.04)
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(score4Pose, parkPose))
                .setLinearHeadingInterpolation(score4Pose.getHeading(), parkPose.getHeading())
                .build();
    }



}



