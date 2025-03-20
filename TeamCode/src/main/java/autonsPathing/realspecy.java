package autonsPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
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
@Autonomous
@Config
public class realspecy extends OpMode {
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



    private final Pose startPose = new Pose(8, 62.325, Math.toRadians(0));
    //32.5 works sometimes
    private final Pose scoreSpecyPose = new Pose(37  , 68.325, Math.toRadians(0));
    private final Pose readytopushPose1 = new Pose(60.97, 34.16, Math.toRadians(0));
    private final Pose pushPose1= new Pose(12, 22.77, Math.toRadians(0));
    private final Pose readytopushPose2 = new Pose(62.63, 19.93, Math.toRadians(0));
    private final Pose pushPose2= new Pose(12, 13.00, Math.toRadians(0));
    private final Pose readytopushPose3 = new Pose(62.630, 10.10, Math.toRadians(0));
    private final Pose pushPose3= new Pose(13, 11.10, Math.toRadians(0));
    private final Pose pickupspecy= new Pose(13, 35.26, Math.toRadians(0));
    private final Pose pickupspecy42= new Pose(12, 35.26, Math.toRadians(0));

    private final Pose scoreSpecyPose2 = new Pose(36  , 70.325, Math.toRadians(0));
    private final Pose scoreSpecyPose3 = new Pose(38  , 73.325, Math.toRadians(0));
    private final Pose scoreSpecyPose4 = new Pose(37  , 75.325, Math.toRadians(0));


    private PathChain scoreSpecy,readytopush1,push1, readytopush2,push2, readytopush3,push3,pushsamplesatonce, scorepath2, pickupspecy2, scorepath3,pickupspecy3, scorepath4, pickupspecy4,scorepath5;

    public void buildPaths() {
        scoreSpecy = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scoreSpecyPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scoreSpecyPose.getHeading())
                .setPathEndTimeoutConstraint(2)
                .build();

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        readytopush1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scoreSpecyPose),
                        new Point(0.237, 46.260, Point.CARTESIAN),
                        new Point(13.997, 29.654, Point.CARTESIAN),
                        new Point(readytopushPose1)))
                .setLinearHeadingInterpolation(scoreSpecyPose.getHeading(), readytopushPose1.getHeading())
                .setPathEndTimeoutConstraint(2)
                .build();



                push1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(readytopushPose1),
                        new Point(69.035, 21.351, Point.CARTESIAN),
                        new Point(15.000, 22.537, Point.CARTESIAN),
                        new Point(pushPose1)))
                .setLinearHeadingInterpolation(readytopushPose1.getHeading(), pushPose1.getHeading())
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
                                new Point(22.774, 71.170, Point.CARTESIAN),
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
                                new Point(scoreSpecyPose3)))
                        .setLinearHeadingInterpolation(pickupspecy.getHeading(), scoreSpecyPose3.getHeading())
                        .build();
        pickupspecy3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scoreSpecyPose3),
                        new Point(22.774, 71.170, Point.CARTESIAN),
                        new Point(pickupspecy)))
                .setLinearHeadingInterpolation(scoreSpecyPose3.getHeading(), pickupspecy.getHeading())
                .build();
        pickupspecy4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scoreSpecyPose4),
                        new Point(22.774, 71.170, Point.CARTESIAN),
                        new Point(pickupspecy42)))
                .setLinearHeadingInterpolation(scoreSpecyPose4.getHeading(), pickupspecy42.getHeading())
                .build();
        scorepath4= follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickupspecy),
                        new Point(22.774, 71.170, Point.CARTESIAN),
                        new Point(scoreSpecyPose4)))
                .setLinearHeadingInterpolation(pickupspecy.getHeading(), scoreSpecyPose4.getHeading())
                .build();



    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
               if(opmodeTimer.getElapsedTime()>200) {


                   follower.followPath(scoreSpecy, true);
                   setPathState(99);
               }
                break;
            case 99:
                if(!follower.isBusy()) {
                    target = 1300;
                    setPathState(100);
                }
                break;
            case 100:
                if(Slides.left.getCurrentPosition()>1250){
                    hand.setPosition(0);
                    //work
                    wrist.setPosition(.5);
                    pivotR.setPosition(.3);
                    pivotR.setPosition(.3);
                    setPathState(98);
                }
                break;
            case 98:
                    target=0;
                    follower.followPath(readytopush1,true);
                    setPathState(1);

                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */
                pivotR.setPosition(.5);
                pivotL.setPosition(.5);

                targetArm=-10;

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    follower.followPath(push1,true);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                     setPathState(2);

                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    follower.followPath(readytopush2,true);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
                    follower.followPath(push2,true);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    follower.followPath(readytopush3,true);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
                    hand.setPosition(0);
                    wrist.setPosition(.45);
                    pivotR.setPosition(.2);
                    pivotR.setPosition(.2);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    setPathState(60);
                }
                break;
            case 60:
                follower.followPath(push3,true);
                setPathState(6);
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    //follower.followPath(scorepath2,true);
                    timer.reset();
                    setPathState(65);
                }
                break;
            case  65:
                hand.setPosition(1);
                if(timer.seconds()>.9){
                    pivotR.setPosition(.15);
                    pivotL.setPosition(.15);
                    wrist.setPosition(.2);
                    turn.setPosition(.67);
                    setPathState(89);

                }
                break;
            case 89:
                targetArm = 1000;
                target = 700;
                setPathState(78);

                break;
            case 78:
                follower.followPath(scorepath2,true);
                setPathState(500);
                break;
            case 500:
                if(!follower.isBusy()) {
                    target = 1300;
                    setPathState(501);
                }
                break;
            case 501:
                if(Slides.left.getCurrentPosition()>1250){
                    hand.setPosition(0);
                    //work
                    wrist.setPosition(.5);
                    pivotR.setPosition(.3);
                    pivotR.setPosition(.3);
                    setPathState(502);
                }
                break;
            case 502:
                target=0;
                targetArm=0;
                hand.setPosition(0);
                wrist.setPosition(.45);
                pivotR.setPosition(.2);
                pivotR.setPosition(.2);
                setPathState(503);

                break;
            case 503:
                follower.followPath(pickupspecy2,true);
                setPathState(504);
                break;
            case 504:
                if(!follower.isBusy()){
                   timer.reset();
                    setPathState(505);

                }
                break;
            case 505:
                hand.setPosition(1);
                if(timer.seconds()>.9){
                    pivotR.setPosition(.15);
                    pivotL.setPosition(.15);
                    wrist.setPosition(.2);
                    turn.setPosition(.67);
                    setPathState(506);
                }
                break;
            case 506:
                targetArm = 1000;
                target = 600;
                setPathState(507);

                break;
            case 507:
                follower.followPath(scorepath3,true);
                setPathState(508);
                break;
            case 508:
                if(!follower.isBusy()) {
                    timer.reset();
                    setPathState(600);
                }
                break;
            case 600:
                if(timer.seconds()>.5) {
                    target = 1300;
                    setPathState(509);
                }

                break;
            case 509:
                if(Slides.left.getCurrentPosition()>1250){
                    hand.setPosition(0);
                    //work
                    wrist.setPosition(.5);
                    pivotR.setPosition(.3);
                    pivotR.setPosition(.3);
                    setPathState(511);
                }
                break;
            case 511:
                target=0;
                targetArm=0;
                hand.setPosition(0);
                wrist.setPosition(.45);
                pivotR.setPosition(.2);
                pivotR.setPosition(.2);
                setPathState(512);

                break;
            case 512:
                follower.followPath(pickupspecy3,true);
                setPathState(514);
                break;
            case 514:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    //follower.followPath(scorepath2,true);
                    timer.reset();
                    setPathState(515);
                }
                break;
            case  515:
                hand.setPosition(1);
                if(timer.seconds()>.9){
                    pivotR.setPosition(.15);
                    pivotL.setPosition(.15);
                    wrist.setPosition(.2);
                    turn.setPosition(.67);
                    setPathState(516);

                }
                break;
            case 516:
                targetArm = 1000;
                target = 600;
                setPathState(518);

                break;
            case 518:
                follower.followPath(scorepath4,true);
                setPathState(519);
                break;
            case 519:
                if(!follower.isBusy()) {
                    target = 1300;
                    setPathState(520);
                }
                break;
            case 520:
                if(Slides.left.getCurrentPosition()>1250){
                    hand.setPosition(0);
                    //work
                    wrist.setPosition(.5);
                    pivotR.setPosition(.3);
                    pivotR.setPosition(.3);
                    setPathState(521);
                }
                break;
            case 521:
                target=0;
                targetArm=0;
                hand.setPosition(0);
                wrist.setPosition(.45);
                pivotR.setPosition(.2);
                pivotR.setPosition(.2);
                setPathState(522);

                break;
            case 522:
                follower.followPath(pickupspecy4,true);
                setPathState(-1);
                break;
            case 523:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    //follower.followPath(scorepath2,true);
                    timer.reset();
                    setPathState(524);
                }
                break;
            case  524:
                hand.setPosition(1);
                if(timer.seconds()>.9){
                    pivotR.setPosition(.15);
                    pivotL.setPosition(.15);
                    wrist.setPosition(.2);
                    turn.setPosition(.67);
                    setPathState(525);

                }
                break;
            case 525:
                targetArm = 1000;
                target = 600;
                setPathState(526);

                break;
            case 527:
                follower.followPath(scorepath4,true);
                setPathState(528);
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
        pivotR.setPosition(.15);
        pivotL.setPosition(.15);
        wrist.setPosition(.2);
        turn.setPosition(.67);
        targetArm = 11000;
        target = 700;

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