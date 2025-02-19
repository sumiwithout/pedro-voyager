package TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@TeleOp
public class testingTelewithpp extends LinearOpMode {
    private static final int MAX_SLDES_POSITION = 2000;


    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);

    private final Pose scoreSpecyPose2 = new Pose(37.5, 66, Math.toRadians(0));
    private final Pose pickupspecy= new Pose(12, 32.26, Math.toRadians(0));



    @Override
    public void runOpMode() throws InterruptedException {

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
       PathChain scorepath3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickupspecy),
                        new Point(22.774, 71.170, Point.CARTESIAN),
                        new Point(scoreSpecyPose2)))
                .setLinearHeadingInterpolation(pickupspecy.getHeading(), scoreSpecyPose2.getHeading())
                .build();
        double waitTime4 = .6;
        ElapsedTime waitTimer4 = new ElapsedTime();
        double waitTime5 = .1;
        ElapsedTime waitTimer5 = new ElapsedTime();
        double waitTime6 = .2;
        ElapsedTime waitTimer6 = new ElapsedTime();



        // the extended gamepad object
        GamepadEx driverOp = new GamepadEx(gamepad1);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        follower.startTeleopDrive();
        while (!isStopRequested() && opModeIsActive()) {

            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
            follower.update();
            if(gamepad1.a){
                follower.setPose(pickupspecy);
                follower.followPath(scorepath3, true);
                follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
            }

            /* Telemetry Outputs of our Follower */
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

            /* Update Telemetry to the Driver Hub */
            telemetry.update();

            telemetry.update();
        }
    }

    }


