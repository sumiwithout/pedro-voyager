package autonsPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
@Autonomous(name = "Teesting Paths", group = "Examples")
public class specimenpaths extends OpMode {
   //slides
    private PIDController controller;
    private static double p = 0.011, i = 0.003, d = 0.00001;
    private static int target = 0;
    private  slides Slides;
    //arm
    private PIDController controllerarm;

    public static double pa = 0.00239, ia = 0, da = 0.00007, fa = 0.2;
    private final double ticks_in_degrees = 700 / 180.0;
    public static int targetArm = 0;
    private  pivot arm;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private ElapsedTime timer;
    double wait = .5;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(8, 62.325, Math.toRadians(0));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(42.000, 62.325, Math.toRadians(0));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose push = new Pose(10.000,6.000, Math.toRadians(0));


        //(10.000, 6.000, Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose drop1 = new Pose(24.434925864909392, 23.723228995057653, Math.toRadians(-22));
    private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(-175));



    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(60, 98, Math.toRadians(90));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private PathChain scorePreload, grabPickup1,dropsample1, puahsamples,grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;


    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        puahsamples = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose),
                new Point(27.044, 57.173, Point.CARTESIAN),
                new Point(20.165, 37.483, Point.CARTESIAN),
//                new Point(56.224, 34.399, Point.CARTESIAN), //repeat
//                        new Point(56.224, 34.399, Point.CARTESIAN),
//                        new Point(56.461, 15.657, Point.CARTESIAN),
//                        new Point(10.000, 21.825, Point.CARTESIAN),
//                        new Point(10.000, 21.825, Point.CARTESIAN),
//                        new Point(56.699, 23.012, Point.CARTESIAN),
//                        new Point(57.173, 12.336, Point.CARTESIAN),
//                        new Point(57.173, 12.336, Point.CARTESIAN),
//                        new Point(10.000, 12.573, Point.CARTESIAN),
//                        new Point(10.000, 12.573, Point.CARTESIAN),
//                        new Point(35.110, 15.183, Point.CARTESIAN),
//                        new Point(57.885, 5.931, Point.CARTESIAN),
//                        new Point(57.885, 5.931, Point.CARTESIAN),
                        new Point(push)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), push.getHeading())
                .build();
        dropsample1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(push),
                        new Point(58.596, 26.570, Point.CARTESIAN),
                        new Point(drop1)
                ))
                .setLinearHeadingInterpolation(push.getHeading(), drop1.getHeading())
                .build();
        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        scorePickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
//                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
//                .build();
//
//        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        grabPickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
//                .build();
//
//        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        scorePickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
//                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
//                .build();


    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                targetArm =1000;
                target=600;
                follower.followPath(scorePreload);

                    setPathState(99);
                    timer.reset();
                break;
            case 99:
if(!follower.isBusy()) {
    target = 0;
    setPathState(98);
}
break;
            case 98:

                if(Slides.left.getCurrentPosition() < 100){
                    targetArm=0;
                    follower.followPath(puahsamples,true);
                    //setPathState(1);
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
                    follower.followPath(dropsample1,true);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                   // setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(dropsample1,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(dropsample1,true);
                    //setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup2,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup3,true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
//                    follower.followPath(park,true);
//                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Level 1 Ascent */

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
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
        private void update(int targetArm) {
            controllerarm.setPID(pa, ia, da);
            int armPos = armL.getCurrentPosition();

            double pid = controllerarm.calculate(armPos, targetArm);
            double ff = Math.cos(Math.toRadians(targetArm / ticks_in_degrees)) * fa;

            double powerPID = pid + ff;

            armR.setPower(powerPID);
            armL.setPower(powerPID);
        }
    }
    private class slides {

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

