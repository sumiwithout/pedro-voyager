package TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
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

@Config
@TeleOp
public class testingTele extends LinearOpMode {
    private static final int MAX_SLDES_POSITION = 2000;

    public enum buttons {
        up, down, wait, wait2, complete, waittimeer, specimendrop, dropoffspecimen, seccey, retractfromthespecimen, pickupspecemine, pitup, readyforspecy, scorepose, checkifscored, rectractanddown, hangpart1, retractlinearslideforhang, zeroitout, movearm, lowerbucket, lowwchaoer, cheeclifzero, finishhang, balancehang, qaitimeer2, hangpart10, submersible, restofsub, hangpart12, resetarm, highasket1
    }
    DistanceSensor dis;
    private PIDController controller;
    // lift
    public static double p = 0.011, i = 0.003, d = 0.00001;
    public static int target = 0;
    ////////////////////////////////////////
    int count =0;
    //arm
    private PIDController controllerarm;

    public static double pa = 0.00239, ia = 0, da = 0.00007, fa = 0.2;
    private final double ticks_in_degrees = 700 / 180.0;
    public static int targetArm = 0;
    //////////// extebd
    //////////Lservos
    double turnposition = 0;
    private Servo hand,turn,wrist, pivotL, pivotR;
private RevBlinkinLedDriver led;
buttons state = buttons.complete;
 public static double hover = 0.07;

 public static double close = 1, hoverwrist = .7,  clawopen =0, wristbasket = .4,wristnotyet = .5, wristspecimen = .5 , hoverspecimen = .575, wristconfirm = .6;
 public static double pivotdown = 0;
Motor fl,fr,bl,br;



    @Override
    public void runOpMode() throws InterruptedException {

fl= new Motor(hardwareMap, "fl", Motor.GoBILDA.RPM_435);
fl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
fr=   new Motor(hardwareMap, "fr", Motor.GoBILDA.RPM_435);
bl= new Motor(hardwareMap, "bl", Motor.GoBILDA.RPM_435);
br =  new Motor(hardwareMap, "br", Motor.GoBILDA.RPM_435);

        double waitTime4 = .6;
        ElapsedTime waitTimer4 = new ElapsedTime();
        double waitTime5 = .1;
        ElapsedTime waitTimer5 = new ElapsedTime();
        double waitTime6 = .5;
        ElapsedTime waitTimer6 = new ElapsedTime();

        MecanumDrive motors = new MecanumDrive(
               fl,
                fr,
             bl,
               br
        );

        dis = hardwareMap.get(DistanceSensor.class, "dis");
        led = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
        imu.initialize(new IMU.Parameters(RevOrientation));


        // the extended gamepad object
        GamepadEx driverOp = new GamepadEx(gamepad1);
        imu.resetYaw();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        hand = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        turn = hardwareMap.get(Servo.class, "turn");
        pivotL = hardwareMap.get(Servo.class, "pivotL");
        pivotR = hardwareMap.get(Servo.class, "pivotR");
        pivotR.setDirection(Servo.Direction.REVERSE);
        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        pivot pivot = new pivot(hardwareMap);
        pivot.armL.setDirection(DcMotorSimple.Direction.FORWARD);

        pivot.armL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        pivot.armL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        pivot.armR.setDirection(DcMotorSimple.Direction.REVERSE);

        pivot.armR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        pivot.armR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slides slides = new slides(hardwareMap);
        slides.right.setDirection(DcMotorSimple.Direction.FORWARD);

        slides.right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slides.right.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        slides.left.setDirection(DcMotorSimple.Direction.REVERSE);

        slides.left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slides.left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
        gamepad2.setLedColor(128,0,128,120000);
        gamepad1.setLedColor(128,0,128,120000);

        waitForStart();
        hand.setPosition(0);
        wrist.setPosition(.45);
        pivotR.setPosition(.2);
        pivotR.setPosition(.2);
        while (!isStopRequested() && opModeIsActive()) {

            pivot.update(targetArm);
            slides.update(target);


            switch (state) {
                case complete:
                    if(gamepad1.right_trigger>.5){
                        target-=100;
                    }
                    if(gamepad1.start){
                        targetArm=1000;
                        waitTimer5.reset();
                        state = buttons.resetarm;
                    }
                    if(gamepad2.right_trigger>.5){
                        hand.setPosition(0);
                        wrist.setPosition(.45);
                        pivotR.setPosition(.2);
                        pivotR.setPosition(.2);
                        turnposition=0;


                        state = buttons.submersible;

                    }
                    if (gamepad1.x){
                        hand.setPosition(clawopen);

                    }
                    if(gamepad1.dpad_down){
                        targetArm =-940;
                        state = buttons.readyforspecy;
                    }
                    if (gamepad2.dpad_up) {

                        turnposition =.7;
                        targetArm = -940;
                        state = buttons.highasket1;
                    }
                    if (gamepad2.dpad_left) {
                        targetArm = -940;
                        state = buttons.lowerbucket;
                    }


//                 pivotL.setPosition(.3);
                    //pivotR.setPosition(.6);
                    if(gamepad2.y){
                        pivotR.setPosition(1);
                        pivotL.setPosition(1);
                        wrist.setPosition(.3);
                        hand.setPosition(close);
                        state = buttons.hangpart12;

                    }
                    if(gamepad2.right_bumper) {

                        pivotR.setPosition(0.25);
                        pivotL.setPosition(0.25);
                        wrist.setPosition(.5);
                        turnposition = .7;
                        state = buttons.specimendrop;
                    }
                    if(gamepad2.left_bumper){
                        state = buttons.pickupspecemine;

                    }
                    break;
                case resetarm:
                    if(waitTimer5.seconds()>.2){
                        pivot.armL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        pivot.armL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        state = buttons.complete;
                    }
                    break;
                case submersible:
                    target = 1500 ;
                    if(slides.left.getCurrentPosition()>1400){
                        pivotR.setPosition(hover);
                        pivotL.setPosition(hover);
                        wrist.setPosition(hoverwrist);
                        hand.setPosition(clawopen);
                        turnposition=0;
                        state = buttons.restofsub;
                    }



                    break;
                case restofsub:
                    if(gamepad1.a){
                        pivotR.setPosition(0.01);
                        pivotL.setPosition(0.01);
                        hand.setPosition(close);
                        wrist.setPosition(wristconfirm);
                    }
                    if(gamepad1.x ){
                        pivotR.setPosition(hover);
                        pivotL.setPosition(hover);
                        wrist.setPosition(hoverwrist);
                        hand.setPosition(clawopen);
                    }


                    if(gamepad1.b) {
                        turnposition = 0;
                        pivotR.setPosition(.25);
                        pivotL.setPosition(.25);
                        wrist.setPosition(wristnotyet);
                        hand.setPosition(close);
                        target = 0;
                        state = buttons.complete;
                    }
                    break;
                case hangpart12:
                    target = 800;
                    if(slides.left.getCurrentPosition()>700){
                        state = buttons.hangpart1;
                    }
                    break;
                case hangpart1:
                        if(gamepad2.b){
                            targetArm =-900;
                            state = buttons.retractlinearslideforhang;
                        }
                    break;
                case retractlinearslideforhang:
                    if(gamepad2.y){
                        target=0;
                        pivotL.setPosition(1);
                        pivotR.setPosition(1);
                        state = buttons.finishhang;
                    }
                    break;
                case finishhang:
                    if(gamepad2.b){
                        targetArm =0;
                        state = buttons.complete;
                    }
                    break;
                case hangpart10:
                    if(gamepad2.b){
                        target=780;
                        state = buttons.hangpart1;
                    }
                    break;
                case zeroitout:
                    target=0;
                    if(slides.left.getCurrentPosition()<100){
                        pivotR.setPosition(.4);
                        pivotL.setPosition(.17);
                        wrist.setPosition(.8);
                        hand.setPosition(close);
                        state = buttons.movearm;
                    }
                    break;
                case movearm:
                    targetArm=0;
                    state = buttons.complete;

                    break;
                case pickupspecemine:


                    hand.setPosition(0);
                    wrist.setPosition(.45);
                    pivotR.setPosition(.2);
                    pivotR.setPosition(.2);
                        turnposition=.7;
                        state = buttons.pitup;

                    break;
                case pitup:

                    if(gamepad1.a){
                        hand.setPosition(close);
                    }
                    if(gamepad1.x){
                        hand.setPosition(clawopen);
                    }

                    if(gamepad1.b){
                        turnposition=0;
                        pivotR.setPosition(0.45);
                        pivotL.setPosition(0.45);
                       wrist.setPosition(wristspecimen);
                        targetArm =-940;
                        state = buttons.readyforspecy;
                    }
                    if(gamepad1.y){
                        target =0;
                        turnposition=.7;
                        pivotR.setPosition(.5);
                        pivotL.setPosition(.5);
                        wrist.setPosition(1);
                        state = buttons.cheeclifzero;
                    }
                    break;
                case cheeclifzero:
                    targetArm =-890;
                    if(slides.left.getCurrentPosition()<100){
                        state = buttons.lowwchaoer;
                    }
                    break;
                case lowwchaoer:
                    if(pivot.armL.getCurrentPosition()<-700){
                        target=1000;
                        state = buttons.scorepose;
                    }
                    break;
                case readyforspecy:
                    if(pivot.armL.getCurrentPosition()<-700){
                        target=800;
                    state = buttons.scorepose;
                    }
                    break;
                case scorepose:
                    if(slides.left.getCurrentPosition()>target-200){
                        state = buttons.checkifscored;
                    }

                    break;
                case checkifscored:
                    if(gamepad1.x){
                        hand.setPosition(clawopen);
                    }
                    if(gamepad1.dpad_up){
                        pivotR.setPosition(1);
                        pivotL.setPosition(1);
                    }
                    if(gamepad1.dpad_right){
                        pivotR.setPosition(0.45);
                        pivotL.setPosition(0.45);
                    }

                    if(gamepad1.b){
                        target=0;
                        pivotR.setPosition(.45);
                        pivotL.setPosition(.22);
                        wrist.setPosition(.8);
                        hand.setPosition(close);
                        state = buttons.rectractanddown;
                    }


                    break;
                case rectractanddown:
                    if(slides.left.getCurrentPosition()<100){
                        targetArm = 0;
                        state = buttons.complete;
                    }


                    break;
                case specimendrop:
                        target=1900;
                        state = buttons.dropoffspecimen;

                    break;
                case dropoffspecimen:

                        if(gamepad1.x){
                            hand.setPosition(clawopen);
                            waitTimer5.reset();
                            state = buttons.seccey;
                        }

                        break;
                case seccey:
                    if(waitTime5<= waitTimer5.seconds()){
                        state = buttons.retractfromthespecimen;

                    }
                    break;
                case retractfromthespecimen:
                    target=0;
                    state = buttons.complete;

                    break;
                case highasket1:
                    if(pivot.armL.getCurrentPosition()<-700) {
                        target = 2200;
                        state = buttons.up;
                    }
                    break;
                case lowerbucket:
                    if(pivot.armL.getCurrentPosition()<-700) {
                        turnposition=0;

                        target = 2000;
                        state = buttons.up;}

                    break;
                case up:
                    state = buttons.wait;
                    break;

                case wait:
                    if (gamepad1.x) {
                        wrist.setPosition(wristbasket);
                        hand.setPosition(clawopen);
                        waitTimer6.reset();
                        state = buttons.qaitimeer2 ;
                    }
                    break;
                case qaitimeer2:
                    if(waitTimer6.seconds()>= waitTime6){
                        waitTimer4.reset();

                        wrist.setPosition(wristnotyet);
                        state = buttons.waittimeer;


                    }


                    break;

                case waittimeer:
                    wrist.setPosition(wristnotyet);
                    if(waitTimer4.seconds()>= waitTime4){

                            waitTimer4.reset();
                            state= buttons.wait2;
                        }
                        break;
                case wait2:
                    target = 0;
                    if(slides.left.getCurrentPosition()<200) {
                        turnposition=.7;
                        targetArm =0;
                        state = buttons.complete;
                    }
                    break;
                case down:
                    if(slides.left.getCurrentPosition()<100) {
                        targetArm = -890;
                        state = buttons.complete;
                    }
                    break;
            }


            if(gamepad2.x){
                targetArm =0;
                turnposition=.7;
                pivotR.setPosition(.45);
                pivotL.setPosition(.22);
                wrist.setPosition(.8);
                hand.setPosition(close);
                state = buttons.complete;
            }
            if(gamepad2.dpad_down && pivot.armL.getCurrentPosition()>-200){
                target=0;
                pivotR.setPosition(.45);
                pivotL.setPosition(.22);
                wrist.setPosition(.8);
                state = buttons.complete;
            }
            else if (gamepad2.dpad_down && pivot.armL.getCurrentPosition()<-200){
                target=0;
            }
            if(gamepad2.start){
                slides.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slides.left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if(gamepad1.y){
                imu.resetYaw();
            }
            if(gamepad2.a){
                targetArm-=10;
            }

            if(gamepad1.left_bumper){
                turnposition=  turnposition<1? turnposition+.03: 1;
            }
            if(gamepad1.right_bumper){
                turnposition = turnposition>0? turnposition-.03:0;
            }
            turn.setPosition(turnposition);
            motors.driveFieldCentric(
                    -driverOp.getLeftX(),
                    -driverOp.getLeftY(),
                    -gamepad1.right_stick_x,
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),
                    false
            );



            telemetry.addData("linearslides position", slides.right.getCurrentPosition());
            telemetry.addData("linearslides position", slides.left.getCurrentPosition());
            telemetry.addData("linearslides target ", target);
            telemetry.addData("ArmPos", pivot.armL.getCurrentPosition());
            telemetry.addData("turnpostion", turnposition);
            telemetry.addData("state", state);
            telemetry.update();
        }
    }
    public class pivot {

        public  DcMotorEx armL;
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

