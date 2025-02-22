package TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class servotest extends LinearOpMode {
    public Servo hand, turn, wrist, pivotL, pivotR;
    public static double hand1, turn1, wrist1, pivotL1, pivotR1;
    public static int test = 0;

    public void runOpMode() throws InterruptedException {
        hand = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        turn = hardwareMap.get(Servo.class, "turn");
        pivotL = hardwareMap.get(Servo.class, "pivotL");
        pivotR = hardwareMap.get(Servo.class, "pivotR");

        waitForStart();
        while (!isStopRequested()) {
            if (test == 0) {
                pivotR.setPosition(pivotR1);
                 // hover left .3
                // pivot arm .6
                //specy drop .4
                //  pivot r .6



                //pivot r score .45
                //pivot l scorre postioin is 22
            } else if (test == 1) {
                pivotL.setPosition(pivotL1);
                // hoer postionni is .05
                // sore postion .22
                // pivot before score .3
                //drorp scpey .2
                // .3
                // hover left.33
                //grab 0
            } else if (test == 2) {
                turn.setPosition(turn1);

            } else if (test == 3) {
                hand.setPosition(hand1);
                //grab 1
                // open 0
            } else if (test == 4) {
                wrist.setPosition(wrist1);
//.45 for speciman score

            } else if (test == 5) {
                pivotR.setDirection(Servo.Direction.REVERSE);



            }
            else if (test ==6){
                pivotR.setPosition(pivotR1);
                pivotL.setPosition(pivotR1);
            }



        }
    }
}
