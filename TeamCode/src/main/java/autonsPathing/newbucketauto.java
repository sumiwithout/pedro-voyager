package autonsPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "wucket", group = "Examples")
@Config
public class newbucketauto extends OpMode {
    //slides
    private PIDController controller;
    public static double p = 0.011, i = 0.003, d = 0.00001;
    public static int target = 0;
    private slides Slides;
    //arm
    private PIDController controllerarm;

    public static double pa = 0.00239, ia = 0, da = 0.00007, fa = 0.2;
    private final double ticks_in_degrees = 700 / 180.0;
    public static int targetArm = 0;
    private pivot arm;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    ElapsedTime timer;
    double wait = .5;
    private Servo hand,turn,wrist, pivotL, pivotR;

    private int pathState;

public static double head=210;
public static double turning = .6;
    private final Pose startPose = new Pose(8, 104.38220757825371, Math.toRadians(90));
    //32.5 works sometimes
    //30
    private final Pose scoresmple = new Pose(15 , 127, Math.toRadians(120));
    private final Pose scoresmple2 = new Pose(15 , 127, Math.toRadians(120));
    private final Pose scoresmple2back = new Pose(17 , 127, Math.toRadians(120));
    private final Pose scoresmple2back2 = new Pose(17 , 127, Math.toRadians(120));
    private final Pose scoresmpleback = new Pose(17 , 127, Math.toRadians(120));
    private final Pose scoresmple2back22 = new Pose(71 , 90, Math.toRadians(120));

    private final Pose pickup1 = new Pose(24.48, 122, Math.toRadians(180));
    private final Pose pickup2 = new Pose(27.48, 129, Math.toRadians(180));
    private final Pose pickup3 = new Pose(28.7, 129.5, Math.toRadians(200));

    private final Pose park = new Pose(28.7, 129.5, Math.toRadians(200));


    private final Pose pushPose1= new Pose(12, 22.77, Math.toRadians(0));
    private final Pose readytopushPose2 = new Pose(62.63, 19.93, Math.toRadians(0));
    private final Pose pushPose2= new Pose(12, 13.00, Math.toRadians(0));
    private final Pose readytopushPose3 = new Pose(62.630, 10.10, Math.toRadians(0));
    private final Pose pushPose3= new Pose(13.5, 10.10, Math.toRadians(0));
    private final Pose scoreSpecyPose2 = new Pose(35, 66, Math.toRadians(0));
    private final Pose scoreSpecyPose3 = new Pose(37.5, 64, Math.toRadians(0));
    private final Pose scoreSpecyPose4 = new Pose(37.5, 62, Math.toRadians(0));
    private final Pose scoreSpecyPose5 = new Pose(37.5, 60, Math.toRadians(0));

    private final Pose pickupspecy= new Pose(12, 32.26, Math.toRadians(0));

    private PathChain scoreSpecy,parking, scoreanothersampleback5,scoreSpecyback,scoreanothersampleback4, scoresample4,pickupsample3,scoreanothersampleback3,scoresample3,pickupsample2,scoreanothersampleback, grabsample,push1,scoreanothersample, readytopush2,push2, readytopush3,push3,pushsamplesatonce, scorepath2, pickupspecy2, scorepath3,pickupspecy3, scorepath4, pickupspecy4,scorepath5;

    public void buildPaths() {
        scoreSpecy = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scoresmple)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scoresmple.getHeading())
                .setPathEndTimeoutConstraint(2)
                .build();
        scoreSpecyback  = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoresmple), new Point(scoresmpleback)))
                .setLinearHeadingInterpolation(scoresmple.getHeading(), scoresmpleback.getHeading())
                .setPathEndTimeoutConstraint(2)
                .build();

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabsample = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoresmpleback),
                        new Point(pickup1)))
                .setLinearHeadingInterpolation(scoresmpleback.getHeading(), pickup1.getHeading())
                .setPathEndTimeoutConstraint(3)
                .build();
        scoreanothersample = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickup1),
                        new Point(26.570, 129.054, Point.CARTESIAN),
                        new Point(scoresmple2)))
                .setLinearHeadingInterpolation(pickup1.getHeading(), scoresmple2.getHeading())
                .setPathEndTimeoutConstraint(1)
                .build();
        scoreanothersampleback = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoresmple2),
                        new Point(scoresmple2back)))
                .setLinearHeadingInterpolation(scoresmple2.getHeading(), scoresmple2back.getHeading())
                .setPathEndTimeoutConstraint(1)
                .build();
        pickupsample2 =follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoresmple2back),
                        new Point(pickup2)))
                .setLinearHeadingInterpolation(scoresmple2back.getHeading(), pickup2.getHeading())
                .setPathEndTimeoutConstraint(3)
                .build();
        scoresample3 =follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2),
                        new Point(scoresmple2)))
                .setLinearHeadingInterpolation(pickup2.getHeading(), scoresmple2.getHeading())
                .setPathEndTimeoutConstraint(3)
                .build();
        scoreanothersampleback3= follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoresmple2),
                        new Point(scoresmple2back2)))
                .setLinearHeadingInterpolation(scoresmple2.getHeading(), scoresmple2back2.getHeading())
                .setPathEndTimeoutConstraint(1)
                .build();
        pickupsample3 =follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoresmple2back2),
                        new Point(pickup3)))
                .setLinearHeadingInterpolation(scoresmple2back2.getHeading(), pickup3.getHeading())
                .setPathEndTimeoutConstraint(3)
                .build();
        scoresample4 =follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickup3),
                        new Point(35.110, 122.412, Point.CARTESIAN),
                        new Point(scoresmple2)))
                .setLinearHeadingInterpolation(pickup3.getHeading(), scoresmple2.getHeading())
                .setPathEndTimeoutConstraint(3)
                .build();
        scoreanothersampleback4= follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoresmple2),
                        new Point(scoresmple2back2)))
                .setLinearHeadingInterpolation(scoresmple2.getHeading(), scoresmple2back2.getHeading())
                .setPathEndTimeoutConstraint(1)
                .build();
        scoreanothersampleback5= follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoresmple2),
                        new Point(scoresmple2back22)))
                .setLinearHeadingInterpolation(scoresmple2.getHeading(), scoresmple2back22.getHeading())
                .setPathEndTimeoutConstraint(1)

                .build();
        parking= follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scoresmple2back22),
                        new Point(59.308, 120.514, Point.CARTESIAN),
                        new Point(park)))
                .setLinearHeadingInterpolation(scoresmple2back22.getHeading(), park.getHeading())
                .setPathEndTimeoutConstraint(1)
                .build();

                push1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickup1),
                        new Point(69.035, 21.351, Point.CARTESIAN),
                        new Point(15.000, 22.537, Point.CARTESIAN),
                        new Point(pushPose1)))
                .setLinearHeadingInterpolation(pickup1.getHeading(), pushPose1.getHeading())
                        .setPathEndTimeoutConstraint(2)
                        .build();
                readytopush2 = follower.pathBuilder()
               .addPath(new BezierCurve(new Point(pushPose1),
                       new Point(55.987, 31.315, Point.CARTESIAN),
                        new Point(readytopushPose2)))
                .setLinearHeadingInterpolation(pushPose1.getHeading(), readytopushPose2.getHeading())
                        .setPathEndTimeoutConstraint(2)
                .build();
                push2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(readytopushPose2),
                        new Point(72.119, 11.387, Point.CARTESIAN),
                        new Point(14.000, 13.000, Point.CARTESIAN),
                        new Point(pushPose2)))
                        .setLinearHeadingInterpolation(readytopushPose2.getHeading(), pushPose2.getHeading())
                        .setPathEndTimeoutConstraint(2)
                .build();
        readytopush3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pushPose2),
                        new Point(55.980, 21.300, Point.CARTESIAN),
                        new Point(readytopushPose3)))
                .setLinearHeadingInterpolation(pushPose2.getHeading(), readytopushPose3.getHeading())
                        .setPathEndTimeoutConstraint(2)
                .build();
                push3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(readytopushPose3),
                        new Point(pushPose3)))
                .setLinearHeadingInterpolation(readytopushPose3.getHeading(), pushPose3.getHeading())
                .setPathEndTimeoutConstraint(20)
                .build();
                scorepath2 = follower.pathBuilder()
                        .addPath(new BezierCurve(new Point(pushPose3),
                                new Point(22.063, 69.272, Point.CARTESIAN),
                                new Point(scoreSpecyPose2)))
                .setLinearHeadingInterpolation(pushPose3.getHeading(), scoreSpecyPose2.getHeading())
                        .build();
                pickupspecy2 = follower.pathBuilder()
                        .addPath(new BezierCurve(new Point(scoreSpecyPose2),
                                new Point(22.774, 71.170, Point.CARTESIAN),
                                new Point(pickupspecy)))
                        .setLinearHeadingInterpolation(scoreSpecyPose2.getHeading(), pickupspecy.getHeading())
                        .build();
                scorepath3 = follower.pathBuilder()
                        .addPath(new BezierCurve(new Point(pickupspecy),
                                new Point(22.774, 71.170, Point.CARTESIAN),
                                new Point(scoreSpecyPose2)))
                        .setLinearHeadingInterpolation(pickupspecy.getHeading(), scoreSpecyPose2.getHeading())
                        .build();
        pickupspecy3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scoresmple),
                        new Point(22.774, 71.170, Point.CARTESIAN),
                        new Point(pickupspecy)))
                .setLinearHeadingInterpolation(scoresmple.getHeading(), pickupspecy.getHeading())
                .build();
        scorepath4= follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickupspecy),
                        new Point(22.774, 71.170, Point.CARTESIAN),
                        new Point(scoresmple)))
                .setLinearHeadingInterpolation(pickupspecy.getHeading(), scoresmple.getHeading())
                .build();



    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                if(opmodeTimer.getElapsedTime()>400) {
                   follower.followPath(scoreSpecy, true);

                   setPathState(99);
               }
                break;
            case 99:
                if(!follower.isBusy()) {
                    setPathState(100);
                    timer.reset();
                }
                break;
            case 100:
                if(timer.seconds()>.2){
                    pivotR.setPosition(.35);
                    pivotL.setPosition(.35);
                    timer.reset();
                    setPathState(102);
                }

                break;
            case 102:
                hand.setPosition(0);
                wrist.setPosition(.5);
                if(timer.seconds()>.5){
                    pivotR.setPosition(.2);
                    pivotL.setPosition(.2);
                    setPathState(250);
                }
                break;
            case 250:
                follower.followPath(scoreSpecyback, true);
                setPathState(260);

                break;
            case 260:
                if(!follower.isBusy()){
                    setPathState(200);
                }
            case  200:
                target=0;
                if(Slides.left.getCurrentPosition()<100){
                    targetArm=-20;
                   follower.followPath(grabsample,true);
                    setPathState(1);
                }
                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */



                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    //follower.followPath(push1,true);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                     setPathState(306);

                }
                break;
            case 306:
                target= 2100;
                setPathState(300);

                break;

            case 300:
                if(Slides.left.getCurrentPosition()>2000){
                    pivotR.setPosition(0.01);
                    pivotL.setPosition(0.01);
                    wrist.setPosition(.6);
                    timer.reset();

setPathState(301);
                }
                break;
            case 301:
                if(timer.seconds()>.4){
                    hand.setPosition(1);
                    timer.reset();
                    setPathState(302);
                }
                break;
            case 302:
                if(timer.seconds()>1){
                    hand.setPosition(1);
                    pivotR.setPosition(.2);
                    pivotL.setPosition(.2);
                    wrist.setPosition(0.4);
                    turn.setPosition(.67);
                    setPathState(2345);
                }
                break;
            case 2345:
                target=400;
                if(Slides.left.getCurrentPosition()<500){
                    targetArm =1200;
                    setPathState(2);

                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */

                    /* Grab Sample */
                    target=2200;
                    follower.followPath(scoreanothersample,true);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    setPathState(303);

                break;
            case 303:
                if(!follower.isBusy()) {
                    setPathState(3423);
                }
                break;
            case 3423:
                pivotR.setPosition(.2);
                pivotL.setPosition(.2);
                timer.reset();
                setPathState(1234);

                break;
            case 1234:
                if(timer.seconds()>.2){
                    hand.setPosition(0);
                    wrist.setPosition(.4);
timer.reset();
                    setPathState(22222);
                }
                break;
            case 22222:
                if (timer.seconds()>.3){
                    pivotR.setPosition(.2);
                    pivotL.setPosition(.2);
                    setPathState(3420);

                }
                break;
            case 3420:
                follower.followPath(scoreanothersampleback,true);
                setPathState(3421);

                break;

            case 3421:
                if(!follower.isBusy()){
                    setPathState(109);

                }
                break;
            case 109:
                if(timer.seconds()>.5){
                    hand.setPosition(0);
                    wrist.setPosition(.5);
                    target=0;
                    setPathState(20040);
                }
                break;
            case  20040:

                if(Slides.left.getCurrentPosition()<100){
                    targetArm=-10;
                    //follower.followPath(grabsample,true);
                    setPathState(3);
                }
                break;
            case 3:

                    /* Score Sample */
                    follower.followPath(pickupsample2,true);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    setPathState(4);

                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    //follower.followPath(readytopush3,true);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    setPathState(500);
                }
                break;
            case 500:
                target= 1900;
                setPathState(501);

                break;

            case 501:
                if(Slides.left.getCurrentPosition()>1800){
                    pivotR.setPosition(0.01);
                    pivotL.setPosition(0.01);
                    wrist.setPosition(.6);
                    timer.reset();

                    setPathState(502);
                }
                break;
            case 502:
                if(timer.seconds()>.4){
                    hand.setPosition(1);
                    timer.reset();
                    setPathState(503);
                }
                break;
            case 503:
                if(timer.seconds()>1){
                    hand.setPosition(1);
                    pivotR.setPosition(.2);
                    pivotL.setPosition(.2);
                    wrist.setPosition(0.4);
                    turn.setPosition(.67);
                    setPathState(504);
                }
                break;
            case 504:
                target=400;
                if(Slides.left.getCurrentPosition()<500){
                    targetArm =1200;
                    setPathState(505);

                }
                break;
            case 505:
                target=2200;
                if(Slides.left.getCurrentPosition()>2100){
                    setPathState(506);
                }
                break;
            case 506:
                follower.followPath(scoresample3,true);
                setPathState(507);

                break;
            case 507:
                if(!follower.isBusy()){
                    setPathState(508);
                }
                break;
            case 508:
                pivotR.setPosition(.2);
                pivotL.setPosition(.2);
                wrist.setPosition(.35);
                timer.reset();
                setPathState(509);

                break;
            case 509:
                if(timer.seconds()>1){
                    hand.setPosition(0);
                    wrist.setPosition(.4);

                    setPathState(510);

                }

                break;
            case 510:
                follower.followPath(scoreanothersampleback3,true);
                setPathState(511);

                break;

            case 511:
                if(!follower.isBusy()){
                    setPathState(512);

                }
                break;
            case 512:
                if(timer.seconds()>.5){
                    hand.setPosition(0);
                    wrist.setPosition(.5);
                    target=0;
                    setPathState(513);
                }
                break;
            case  513:

                if(Slides.left.getCurrentPosition()<100){
                    targetArm=-10;
                    //follower.followPath(grabsample,true);
                    setPathState(514);
                }
                break;
            case 514:
                follower.followPath(pickupsample3,true);
                setPathState(515);
                break;
            case 515:
                if(!follower.isBusy()){
                    setPathState(516);
                }
                break;

            case 516:
                target= 2000;
                setPathState(517);

                break;

            case 517:
                if(Slides.left.getCurrentPosition()>1800){
                    pivotR.setPosition(0.01);
                    pivotL.setPosition(0.01);
                    wrist.setPosition(.6);
                    turn.setPosition(.6);
                    timer.reset();

                    setPathState(518);
                }
                break;
            case 518:
                if(timer.seconds()>.4){
                    hand.setPosition(1);
                    timer.reset();
                    setPathState(519);
                }
                break;
            case 519:
                if(timer.seconds()>1){
                    hand.setPosition(1);
                    pivotR.setPosition(.2);
                    pivotL.setPosition(.2);
                    wrist.setPosition(0.4);
                    turn.setPosition(.67);
                    setPathState(520);
                }
                break;
            case 520:
                target=400;
                if(Slides.left.getCurrentPosition()<500){
                    targetArm =1200;
                    setPathState(521);

                }
                break;
            case 521:
                target=2200;
                if(Slides.left.getCurrentPosition()>2100){
                    setPathState(522);
                }
                break;
            case 522:
                follower.followPath(scoresample4,true);
                setPathState(523);

                break;
            case 523:
                if(!follower.isBusy()){
                    setPathState(524);
                }
                break;
            case 524:
                pivotR.setPosition(.2);
                pivotL.setPosition(.2);
                hand.setPosition(0);
                wrist.setPosition(.4);
                timer.reset();
                setPathState(525);

                break;
            case 525:
                if(timer.seconds()>1){
                    pivotR.setPosition(.2);
                    pivotL.setPosition(.2);

                    setPathState(526);

                }

                break;
            case 526:
                follower.followPath(scoreanothersampleback5,true);
                setPathState(527);

                break;

            case 527:
                if(!follower.isBusy()){
                    setPathState(528);

                }
                break;
            case 528:
                if(timer.seconds()>.5){
                    hand.setPosition(0);
                    wrist.setPosition(.5);
                    target=0;
                    setPathState(529);
                } break;
            case 529:
                if(timer.seconds()>.5){
                    hand.setPosition(0);
                    wrist.setPosition(.5);
                    target=0;
                    setPathState(530);
                }
                break;
            case  530:

                if(Slides.left.getCurrentPosition()<100){
                    targetArm=-10;
                    //follower.followPath(grabsample,true);
                    setPathState(531);
                }
                break;
            case 531:
follower.followPath(parking, true);
setPathState(532);
                break;
            case 532:
                if(!follower.isBusy()){
                    setPathState(-1);
                }
                break;

            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
                   follower.followPath(push3,true);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    hand.setPosition(1);

                    timer.reset();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    setPathState(10001);
                }
                break;

            case 10001:
                if(timer.seconds()>.7) {
                    pivotR.setPosition(.15);
                    pivotL.setPosition(.15);
                    wrist.setPosition(.2);
                    turn.setPosition(.67);
                    follower.followPath(scorepath2, true);
                    setPathState(7);

                }
                break;

            case 7:
                targetArm=1200;
                target=300;
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
//                    follower.followPath(park,true);
                    setPathState(10003);
                    timer.reset();

                }
                break;
            case 10003:
                if(timer.seconds()>.4){
                    target=1300;
                    setPathState(10002);

                }
                break;
            case 10002:
                if(Slides.left.getCurrentPosition()>1300){
                    hand.setPosition(0);
                    //work
                    wrist.setPosition(.5);
                    pivotR.setPosition(.25);
                    pivotR.setPosition(.25);
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Level 1 Ascent */
                    //follower.followPath(pickupspecy2,true);
                    //follower.followPath(scorepath3,true);

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    follower.followPath(pickupspecy3,true);
                    setPathState(10);

                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    follower.followPath(scorepath4,true);
                    setPathState(11);

                }
                break;
            case 11:

                if(!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        arm.update(targetArm);
        Slides.update(target);
        follower.update();
        follower.drawOnDashBoard();
        autonomousPathUpdate();


        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        Slides = new slides(hardwareMap);
        timer = new ElapsedTime();
        arm = new pivot(hardwareMap);
        hand = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        turn = hardwareMap.get(Servo.class, "turn");
        pivotL = hardwareMap.get(Servo.class, "pivotL");
        pivotR = hardwareMap.get(Servo.class, "pivotR");
        hand.setPosition(1);
        pivotR.setDirection(Servo.Direction.REVERSE);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        hand.setPosition(1);
        pivotR.setPosition(.2);
        pivotL.setPosition(.2);
        wrist.setPosition(0.4);
        turn.setPosition(.67);
        targetArm = 1200;
        target = 2200;

    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
    public class pivot {

        public DcMotorEx armL;
        public  DcMotorEx armR;
        public pivot(HardwareMap hardwareMap) {
            controllerarm = new PIDController(pa, ia, da);

            armL = hardwareMap.get(DcMotorEx.class, "armL");
            armL.setDirection(DcMotorEx.Direction.REVERSE);
            armR = hardwareMap.get(DcMotorEx.class, "armR");
            armR.setDirection(DcMotorEx.Direction.FORWARD);
        }
        public void update(int targetArm) {
            controllerarm.setPID(pa, ia, da);
            int armPos = armL.getCurrentPosition();

            double pid = controllerarm.calculate(armPos, targetArm);
            double ff = Math.cos(Math.toRadians(targetArm / ticks_in_degrees)) * fa;

            double powerPID = pid + ff;

            armR.setPower(powerPID);
            armL.setPower(powerPID);
        }
    }
    public class slides {

        public DcMotorEx left,right;
        public slides(HardwareMap hardwareMap) {
            controller = new PIDController(p, i, d);

            left = hardwareMap.get(DcMotorEx.class, "left");
            right = hardwareMap.get(DcMotorEx.class, "right");
            left.setDirection(DcMotorEx.Direction.REVERSE);
            right.setDirection(DcMotorEx.Direction.FORWARD);
            controller.setPID(p, i, d);
        }

        public void update(int target) {
            controller.setPID(p, i, d);
            int slidepos = left.getCurrentPosition();

            double pid = controller.calculate(slidepos, target);
//            double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * fa;

//            double powerPID = pid + ff;

            left.setPower(pid);
            right.setPower(pid);
        }
    }
}