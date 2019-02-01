package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

//dab
@Autonomous(name = "testAutoPath", group = "Prototyping")

public class testAutoPath extends LinearOpMode {
    Servo rightLock;// = hardwareMap.servo.get("rightLock");
    Servo leftLock;// = hardwareMap.servo.get("leftLock");
    DcMotor leftWheel1;// = hardwareMap.dcMotor.get("leftWheel");
    DcMotor leftWheel2;// = hardwareMap.dcMotor.get("leftWheel2");
    DcMotor rightWheel1;// = hardwareMap.dcMotor.get("rightWheel");
    DcMotor rightWheel2;// = hardwareMap.dcMotor.get("rightWheel2");
    DcMotor leftArm;// = hardwareMap.dcMotor.get("leftArm");
    DcMotor rightArm;// = hardwareMap.dcMotor.get("rightArm");
    DcMotor armExtendRight;// = hardwareMap.dcMotor.get("armExtend");
    DcMotor armExtendLeft;// = hardwareMap.dcMotor.get("armExtend2");

//    private GoldDetector detector;
//    double alignSize=70;
//    double alignX    = 270;//  (640 / 2) +0; // Center point in X Pixels
//    double alignXMin = alignX - (alignSize / 2); // Min X Pos in pixels
//    double alignXMax = alignX +(alignSize / 2); // Max X pos in pixels
//    double scale = (640)/2;//-alignSize

    private ElapsedTime runtime = new ElapsedTime();
    private final double TICKS_PER_WHEEL_ROTATION = 1120;

    int subStep=0;
    private int step;

    private enum state {TURN_TO_WALL, MOVE_TO_WALL, MOVE_AWAY_FROM_WALL, TURN_TO_DEPOT, MOVE_TO_DEPOT}
    double stage = -4;

    int data[][]={{0,0,160062,0,0,0},{0,0,160062,0,0,0},{0,0,160062,0,0,0},{0,0,160062,0,0,0},{0,0,160062,0,0,0},{0,0,160062,0,0,0},{0,0,160062,0,0,0},{0,0,160062,0,0,0},{0,0,160062,0,0,0},{0,0,160062,0,0,0},{0,0,160062,0,0,0},{0,0,160062,0,0,0},{0,0,160062,0,0,0},{0,0,160062,0,0,0},{0,0,160062,0,0,0},{0,0,160062,0,0,0},{0,0,160062,0,0,0},{-1,0,160062,0,0,0},{-2,0,160062,0,0,0},{-3,0,160062,0,0,0},{-5,0,160062,0,0,0},{-5,2,160062,0,0,0},{-7,4,160062,0,0,0},{-11,8,160000,0,0,0},{-18,14,160000,0,0,0},{-25,22,160000,0,0,0},{-37,36,160000,0,0,0},{-54,52,159937,0,0,0},{-70,68,159937,0,0,0},{-89,89,159812,0,0,0},{-107,110,159562,0,0,0},{-124,129,159437,0,0,0},{-147,155,159062,0,0,0},{-172,187,158562,0,0,0},{-199,219,158062,0,0,0},{-225,251,157437,0,0,0},{-256,288,156812,0,0,0},{-285,326,156125,0,0,0},{-318,374,154812,0,0,0},{-345,418,153312,0,0,0},{-378,470,151250,0,0,0},{-405,511,149875,0,0,0},{-427,549,148250,0,0,0},{-454,596,146375,0,0,0},{-479,646,144125,0,0,0},{-501,698,141187,0,0,0},{-522,745,138500,0,0,0},{-544,797,135625,0,0,0},{-563,846,132937,0,0,0},{-577,889,130062,0,0,0},{-592,935,126687,0,0,0},{-606,980,123437,0,0,0},{-623,1022,120937,0,0,0},{-643,1064,118812,0,0,0},{-662,1101,116687,0,0,0},{-686,1143,114875,0,0,0},{-709,1180,113437,0,0,0},{-736,1223,112062,0,0,0},{-766,1266,110750,0,0,0},{-804,1313,109625,0,0,0},{-845,1357,109062,0,0,0},{-889,1399,109062,0,0,0},{-938,1442,109437,0,0,0},{-985,1479,110000,0,0,0},{-1033,1517,110625,0,0,0},{-1077,1550,111875,0,0,0},{-1123,1579,113437,0,0,0},{-1166,1600,115062,0,0,0},{-1215,1621,117750,0,0,0},{-1258,1635,120750,0,0,0},{-1299,1650,123312,0,0,0},{-1338,1667,125750,0,0,0},{-1383,1695,127750,0,0,0},{-1437,1734,129062,0,0,0},{-1489,1772,130312,0,0,0},{-1540,1809,131937,0,0,0},{-1592,1841,133625,0,0,0},{-1646,1875,135812,0,0,0},{-1715,1916,138687,0,0,0},{-1776,1954,140812,0,0,0},{-1827,1986,142750,0,0,0},{-1875,2014,144562,0,0,0},{-1928,2042,146812,0,0,0},{-1972,2067,148812,0,0,0},{-2015,2090,151000,0,0,0},{-2058,2113,153000,0,0,0},{-2101,2137,154812,0,0,0},{-2132,2154,156250,0,0,0},{-2168,2178,157625,0,0,0},{-2198,2197,158562,0,0,0},{-2225,2218,159312,0,0,0},{-2247,2238,159687,0,0,0},{-2270,2255,160062,0,0,0},{-2286,2270,160187,0,0,0},{-2301,2283,160250,0,0,0},{-2308,2290,160250,0,0,0},{-2311,2294,160250,0,0,0},{-2310,2294,160250,0,0,0},{-2310,2294,160250,0,0,0},{-2310,2294,160250,0,0,0},{-2310,2294,160250,0,0,0},{-2310,2294,160250,0,0,0},{-2310,2294,160250,0,0,0},{-2310,2294,160250,0,0,0},{-2310,2294,160250,0,0,0},{-2310,2294,160250,0,0,0},{-2310,2294,160250,0,0,0},{-2310,2294,160250,0,0,0},{-2310,2292,160250,0,0,0},{-2308,2292,160250,0,0,0},{-2304,2288,160187,0,0,0},{-2299,2283,160187,0,0,0},{-2293,2275,160187,0,0,0},{-2282,2266,160187,0,0,0},{-2269,2253,160187,0,0,0},{-2253,2238,160125,0,0,0},{-2236,2221,160000,0,0,0},{-2217,2203,159875,0,0,0},{-2194,2182,159687,0,0,0},{-2171,2163,159437,0,0,0},{-2143,2137,159062,0,0,0},{-2107,2107,158625,0,0,0},{-2077,2081,158187,0,0,0},{-2039,2051,157500,0,0,0},{-2006,2027,156625,0,0,0},{-1969,2003,155312,0,0,0},{-1932,1980,154062,0,0,0},{-1892,1956,152375,0,0,0},{-1851,1932,150187,0,0,0},{-1802,1906,148062,0,0,0},{-1752,1883,145687,0,0,0},{-1703,1859,143125,0,0,0},{-1651,1835,140375,0,0,0},{-1602,1814,137312,0,0,0},{-1549,1795,134000,0,0,0},{-1498,1779,130562,0,0,0},{-1448,1764,126937,0,0,0},{-1401,1749,123312,0,0,0},{-1348,1734,120375,0,0,0},{-1307,1723,117500,0,0,0},{-1259,1706,114250,0,0,0},{-1218,1691,111437,0,0,0},{-1176,1671,109125,0,0,0},{-1142,1652,107750,0,0,0},{-1107,1630,106312,0,0,0},{-1075,1607,105437,0,0,0},{-1041,1577,104937,0,0,0},{-1009,1546,104812,0,0,0},{-977,1513,104937,0,0,0},{-942,1475,105187,0,0,0},{-907,1434,105750,0,0,0},{-871,1392,106500,0,0,0},{-834,1343,107687,0,0,0},{-804,1298,109000,0,0,0},{-775,1252,110687,0,0,0},{-747,1208,112375,0,0,0},{-716,1157,114500,0,0,0},{-679,1094,116937,0,0,0},{-648,1051,118750,0,0,0},{-617,997,121000,0,0,0},{-590,950,123125,0,0,0},{-563,894,125937,0,0,0},{-536,845,128125,0,0,0},{-515,802,130687,0,0,0},{-492,750,133375,0,0,0},{-475,700,136875,0,0,0},{-462,659,139500,0,0,0},{-445,615,142000,0,0,0},{-430,577,144687,0,0,0},{-407,524,147375,0,0,0},{-386,489,149187,0,0,0},{-360,452,150437,0,0,0},{-332,420,151000,0,0,0},{-300,385,151250,0,0,0},{-266,351,151312,0,0,0},{-236,317,151562,0,0,0},{-208,285,152000,0,0,0},{-183,255,152312,0,0,0},{-159,226,152812,0,0,0},{-136,200,153312,0,0,0},{-118,177,153875,0,0,0},{-98,148,154812,0,0,0},{-84,127,155562,0,0,0},{-68,103,156250,0,0,0},{-54,82,157062,0,0,0},{-40,62,157812,0,0,0},{-29,43,158562,0,0,0},{-17,26,159187,0,0,0},{-9,13,159625,0,0,0},{-1,0,160125,0,0,0},{4,-6,160375,0,0,0},{9,-11,160437,0,0,0},{14,-15,160437,0,0,0},{15,-17,160437,0,0,0},{16,-17,160375,0,0,0},{16,-17,160375,0,0,0},{16,-17,160375,0,0,0},{16,-17,160375,0,0,0},{16,-17,160375,0,0,0},{16,-17,160375,0,0,0},{16,-17,160375,0,0,0},{16,-17,160375,0,0,0},{16,-17,160375,0,0,0},{16,-17,160375,0,0,0},{16,-17,160375,0,0,0},{16,-17,160375,0,0,0},{16,-17,160375,0,0,0},{16,-17,160375,0,0,0},{16,-17,160375,0,0,0},{16,-17,160375,0,0,0},{16,-17,160375,0,0,0}};





    int pos = 0;

    int setL = 0;
    int setR = 0;

    int speedL = 0;
    int speedR = 0;

    double gyro = 0.0;
    int arm = 0;
    int extend = 0;
    int intake = 0;

    int errorR;
    int errorL;

    int sumErrorR;
    int sumErrorL;

    int previousErrorR;
    int previousErrorL;

    //double Kp = .002;
    //double Ki = 0.0001;
    //double Kd = 0.001;

    double Kp = 0.001;
    double Ki = 0.00006;
    double Kd = 0;

    double Kf = 0.009;

    boolean aligned = false;

    String csvData = "leftTargetPos,leftCurrentPos,P,I,D\r\n";

    public void runOpMode() throws InterruptedException {
        rightLock = hardwareMap.servo.get("rightLock");
        leftLock = hardwareMap.servo.get("leftLock");
        leftWheel1 = hardwareMap.dcMotor.get("leftWheel");
        leftWheel2 = hardwareMap.dcMotor.get("leftWheel2");
        rightWheel1 = hardwareMap.dcMotor.get("rightWheel");
        rightWheel2 = hardwareMap.dcMotor.get("rightWheel2");
        leftArm = hardwareMap.dcMotor.get("leftArm");
        rightArm = hardwareMap.dcMotor.get("rightArm");
        armExtendRight = hardwareMap.dcMotor.get("armExtend");
        armExtendLeft = hardwareMap.dcMotor.get("armExtend2");

        //rightWheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightWheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        step = -1;

//        telemetry.addData("Status", "DogeCV 2018.0 - Gold Detector test");
//        // Setup detector
//        detector = new GoldDetector(); // Create detector
//        detector.setAdjustedSize(new Size(480, 270)); // Set detector size
//        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize detector with app context and camera
//        detector.useDefaults(); // Set default detector settings
//        // Optional tuning
//        detector.downscale = 0.4; // How much to downscale the input frames
//
//        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
//        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
//        detector.maxAreaScorer.weight = 0.005;
//
//        detector.ratioScorer.weight = 5;
//        detector.ratioScorer.perfectRatio = 1.0;
//        detector.enable(); // Start detector


        //prepares and resets robot by resetting encoders
        //resetDriveEncoders();

        //prepares robot for arm movement by setting arm motors to go to a certain position-modifiable by user input (the d-pad)
//        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //HSV value array

        //Sends color sensor input values to the phone
        waitForStart();


        rightWheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftWheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {
            telemetry.update();


           // telemetry.addLine()
               //     .addData("step", step);
            telemetry.addLine()
                    .addData("rightWheel1: ", leftWheel1.getCurrentPosition())
                    .addData("rightWheel1: ", rightWheel1.getCurrentPosition());
            //.addData("TICKS_PER_WHEEL_ROTATION: ", TICKS_PER_WHEEL_ROTATION);
          //  telemetry.addData("arm::::::", leftArm.getCurrentPosition());
            //telemetry.addData("armExtend", armExtendLeft.getCurrentPosition());
            //telemetry.addData("armExtend2", armExtendRight.getCurrentPosition());

            if (step == -1) {
                if (runtime.seconds() > .025) {
                    setL = data[pos][1];
                    setR = data[pos][0];
                    speedL =  data[pos+1][1] - data[pos][1];
                    speedR =  data[pos+1][0] - data[pos][0];
                    gyro = ((double)data[pos][2])/1000;
                    arm = data[pos][3];
                    extend = data[pos][4];
                    intake= data[pos][5];
                    pos++;
                    runtime.reset();
                }
                errorR= setR-rightWheel1.getCurrentPosition();
                errorL = setL-leftWheel1.getCurrentPosition();
                double pr;
                double pl;


                double dErrorR = errorR - previousErrorR;
                double dErrorL = errorL - previousErrorL;

                pr = errorR*Kp + Kd*dErrorR + Ki*sumErrorR + Kf*speedR;
                pl = errorL*Kp + Kd*dErrorL + Ki*sumErrorL + Kf*speedL;

                rightWheel1.setPower(pr);
                rightWheel2.setPower(pr);
                leftWheel1.setPower(pl);
                leftWheel2.setPower(pl);

                telemetry.addData("pr",pr);
                telemetry.addData("pl",pl);
                telemetry.addData("errorL:", errorL);
                telemetry.addData("errorR",errorR);


                csvData += setL + ","+leftWheel1.getCurrentPosition()+ "," + errorL*Kp + ","+Ki*sumErrorL +"," + Kd*dErrorL + "," + Kf*speedL + "\r\n";

                sumErrorL += errorL;
                sumErrorR += errorR;

                if(sumErrorL > 1/Ki){
                    sumErrorL = (int) (1/Ki);
                }
                if(sumErrorR > 1/Ki){
                    sumErrorR = (int) (1/Ki);
                }


                previousErrorL = errorL;
                previousErrorR = errorR;


            }
//            if (step == 0) {
//                leftLock.setPosition(.25);
//                rightLock.setPosition(.4);
//                if(leftArm.getCurrentPosition()>-4250) {
//                    //leftArm.setPower(.3);
//                    rightArm.setPower(-.3);
//
//                }
//                else if (leftArm.getCurrentPosition()>-5000){
//                    //leftArm.setPower(.65);
//                    rightArm.setPower(-.65);
//                    if(rightWheel1.getCurrentPosition()>-400) {
//                        leftWheel2.setPower(-.4);
//                        leftWheel1.setPower(-.4);
//                        rightWheel2.setPower(-.4);
//                        rightWheel1.setPower(-.4);
//                    }
//                    else{
//                        leftWheel2.setPower(0);
//                        leftWheel1.setPower(0);
//                        rightWheel2.setPower(0);
//                        rightWheel1.setPower(0);
//                    }
//                }
//                else{
//                    leftWheel2.setPower(0);
//                    leftWheel1.setPower(0);
//                    rightWheel2.setPower(0);
//                    rightWheel1.setPower(0);
//                   // leftArm.setPower(0);
//                    rightArm.setPower(0);
//                    step=1;
//                    runtime.reset();
//                }
//
//            }
//            if(step==1){
//                //leftArm.setPower(-.55);
//                 rightArm.setPower(.55);
//                if(runtime.seconds()>2.5){
//                    leftArm.setPower(0);
//                    rightArm.setPower(0);
//                    runtime.reset();
//                    step=2;
//                    rightWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    leftWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    rightWheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    leftWheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                }
//            }
//            if(step==2) {
//                Rect bestRect = detector.getFoundRect();
//
//                double xPos = bestRect.x + (bestRect.width / 2);
//
//                if (xPos < alignXMax && xPos > alignXMin) {
//                    aligned = true;
//                } else {
//                    aligned = false;
//                }
//                telemetry.addData("aligned ", aligned);
//
//                telemetry.addData("xpos ", xPos);
//                telemetry.addData("amax ", alignXMax);
//                telemetry.addData("amin ", alignXMin);
//                if(!(xPos>0)){
//                    if(runtime.seconds()<4) {
//                        rightWheel1.setPower(0);
//                        rightWheel2.setPower(0);
//                        leftWheel1.setPower(0);
//                        leftWheel2.setPower(0);
//                    }
//                    telemetry.addLine("not detected");
//
//                    if(runtime.seconds()>4 && runtime.seconds()<5){
//                        if(rightWheel1.getCurrentPosition()>-150) {
//                            rightWheel1.setPower(-.4);
//                            rightWheel2.setPower(-.4);
//                            leftWheel1.setPower(.4);
//                            leftWheel2.setPower(.4);
//                        }
//                        else{
//                            rightWheel1.setPower(0);
//                            rightWheel2.setPower(0);
//                            leftWheel1.setPower(0);
//                            leftWheel2.setPower(0);
//                        }
//                    }
//                    if(runtime.seconds()>5 && runtime.seconds()<7){
//                        if(rightWheel1.getCurrentPosition()<150) {
//                            rightWheel1.setPower(.4);
//                            rightWheel2.setPower(.4);
//                            leftWheel1.setPower(-.4);
//                            leftWheel2.setPower(-.4);
//                        }
//                        else{
//                            rightWheel1.setPower(0);
//                            rightWheel2.setPower(0);
//                            leftWheel1.setPower(0);
//                            leftWheel2.setPower(0);
//                        }
//                    }
//                    if(runtime.seconds()>7){
//                        if(rightWheel1.getCurrentPosition()>1) {
//                            rightWheel1.setPower(-.4);
//                            rightWheel2.setPower(-.4);
//                            leftWheel1.setPower(.4);
//                            leftWheel2.setPower(.4);
//                        }
//                        else{
//                            runtime.reset();
//                            step++;
//                            rightWheel1.setPower(0);
//                            rightWheel2.setPower(0);
//                            leftWheel1.setPower(0);
//                            leftWheel2.setPower(0);
//                        }
//
//                    }
//                }
//                else if (xPos > alignXMax) {
//                    double power = ((xPos - alignXMax) / scale) * .12 + .11;
//                    rightWheel1.setPower(-power);
//                    rightWheel2.setPower(-power);
//                    leftWheel1.setPower(power);
//                    leftWheel2.setPower(power);
//                    telemetry.addData("powL: ", power);
//                    telemetry.addLine("turning left");
//                    runtime.reset();
//                } else if (xPos < alignXMin) {
//                    double power = ((alignXMin - xPos) / scale) * .12 + .11 ;
//                    rightWheel1.setPower(power);
//                    rightWheel2.setPower(power);
//                    leftWheel1.setPower(-power);
//                    leftWheel2.setPower(-power);
//                    telemetry.addData("powR: ", power);
//                    telemetry.addLine("turning right");
//                    runtime.reset();
//                } else {
//                    rightWheel1.setPower(0);
//                    rightWheel2.setPower(0);
//                    leftWheel1.setPower(0);
//                    leftWheel2.setPower(0);
//                    telemetry.addLine("found");
//                    telemetry.addData("secks: ", runtime.seconds());
//                    if(runtime.seconds()>.3){
//                        runtime.reset();
//                        step++;
//                        //resetDriveEncoders();
//                    }
//                }
//
//
//            }
//            else if (step==3){
//                if(rightWheel1.getCurrentPosition()<1500) {
//                    leftWheel2.setPower(.6);
//                    leftWheel1.setPower(.6);
//                    rightWheel2.setPower(.6);
//                    rightWheel1.setPower(.6);
//                }
//                else {
//                    rightWheel1.setPower(0);
//                    rightWheel2.setPower(0);
//                    leftWheel1.setPower(0);
//                    leftWheel2.setPower(0);
//                }
//                //stage = 2;
//            }
//
//    /*
////sets the step of where we are in the process to one
//            // step = 1;
////enables the if statement for lander_unlatched
//            lander_unlatched = true;
////moves the robot from the lander to where the samples are
//            if (step == 1) {
////                leftWheel.setPower(0.7);
////                rightWheel.setPower(0.5);
////                leftWheel.setTargetPosition(1621);
////                rightWheel.setTargetPosition(820);
//                step = 2;
//            }
//            if (step == 2) {
////                if (hsvValues[0] > 50) {
////                    leftWheel.setPower(-0.7);
////                    rightWheel.setPower(-0.5);
////                    leftWheel.setTargetPosition(1221);
//                step = 3;
//            }
////            if step 2 is finished, makes the robot turn toward the wall, then drives against the wall.
//            if (step == 3) {
//                moveToDepot();
//            }
//            //1150 Target Units == about 1 foot, 96 Units/inch
////sets the wheel power to 0 to limit movement
////                if (rightWheel.getCurrentPosition() == -rotation) {
//            // rightWheel.setPower(0);
//            //leftWheel.setPower(0);
////                }
////               if (leftWheel.getCurrentPosition() == 100) {
////                    leftWheel.setPower(0);
////                    rightWheel.setPower(0);
////                }
//
//            sample_ready = true;
//
////        if (sample_ready) {
////
//        }
//
//        */
//        }
//    }
//
//    private void resetDriveEncoders() {
//        telemetry.addLine("RESET!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n" +
//                "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
//        leftWheel1.setPower(0);
//        leftWheel2.setPower(0);
//        rightWheel1.setPower(0);
//        rightWheel2.setPower(0);
//        leftWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftWheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightWheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftWheel1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftWheel2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightWheel1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightWheel2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//
//    }
//
//    private void moveToDepot() throws InterruptedException {
//
//        if ((subStep * 2) % 2 == 1) {
//            resetDriveEncoders();
//            subStep += 0.5;
//        } else if (subStep == state.TURN_TO_WALL.ordinal()) {
//            // turns the robot 90 degrees counter clockwise
//            double target = .75;
//
//            leftWheel1.setPower(0.5);
//            leftWheel2.setPower(0.5);
//            rightWheel1.setPower(0.5);
//            rightWheel2.setPower(0.5);
//
//            leftWheel1.setTargetPosition((int) (-target * TICKS_PER_WHEEL_ROTATION));
//            leftWheel2.setTargetPosition((int) (-target * TICKS_PER_WHEEL_ROTATION));
//            rightWheel1.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//            rightWheel2.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//
//            telemetry.addData("delta", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel1.getCurrentPosition())));
//            telemetry.addData("delta", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel2.getCurrentPosition())));
//
//            if (!leftWheel1.isBusy() && !leftWheel2.isBusy() && !rightWheel1.isBusy() && !rightWheel2.isBusy()) {
//                subStep += 0.5;
//            }
//        }
////
//        else if (subStep == state.MOVE_TO_WALL.ordinal()) {
//            // moves the robot forward up against the wall
//
//            double target = 3;
//
//            leftWheel1.setPower(0.5);
//            leftWheel2.setPower(0.5);
//            rightWheel1.setPower(0.5);
//            rightWheel2.setPower(0.5);
//
//            leftWheel1.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//            leftWheel2.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//            rightWheel1.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//            rightWheel2.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//
//            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel1.getCurrentPosition())));
//            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel2.getCurrentPosition())));
//
//
//            if (!leftWheel1.isBusy() && !leftWheel2.isBusy() && !rightWheel1.isBusy() && !rightWheel2.isBusy()) {
//                subStep += 0.5;
//            }
//        } else if (subStep == state.MOVE_AWAY_FROM_WALL.ordinal()) {
//            // moves the robot backward away from the wall
//            double target = -1;
//
//            leftWheel1.setPower(0.5);
//            leftWheel2.setPower(0.5);
//            rightWheel1.setPower(0.5);
//            rightWheel2.setPower(0.5);
//
//            leftWheel1.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//            leftWheel2.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//            rightWheel1.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//            rightWheel2.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//
//            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel1.getCurrentPosition())));
//            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel2.getCurrentPosition())));
//
//
//            if (!leftWheel1.isBusy() && !leftWheel2.isBusy() && !rightWheel1.isBusy() && !leftWheel2.isBusy()) {
//                subStep += 0.5;
//
//            }
//        } else if (subStep == state.TURN_TO_DEPOT.ordinal()) {
//            // turns the robot counter-clockwise to line up with the depot
//            double target = 1;
//
//            leftWheel1.setPower(0.5);
//            leftWheel2.setPower(0.5);
//            rightWheel1.setPower(0.5);
//            rightWheel2.setPower(0.5);
//
//            leftWheel1.setTargetPosition((int) (-target * TICKS_PER_WHEEL_ROTATION));
//            leftWheel2.setTargetPosition((int) (-target * TICKS_PER_WHEEL_ROTATION));
//            rightWheel1.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//            rightWheel2.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//
//            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel1.getCurrentPosition())));
//            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel2.getCurrentPosition())));
//
//
//            if (!leftWheel1.isBusy() && !leftWheel2.isBusy() && !rightWheel1.isBusy() && !rightWheel2.isBusy()) {
//                subStep += 0.5;
//            }
//        } else if (subStep == state.MOVE_TO_DEPOT.ordinal()) {
//            // moves the robot forward up to the depot
//            double target = 5;
//
//            leftWheel1.setPower(0.5);
//            leftWheel2.setPower(0.5);
//            rightWheel1.setPower(0.5);
//            rightWheel2.setPower(0.5);
//
//            leftWheel1.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//            leftWheel2.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//            rightWheel1.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//            rightWheel2.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
//
//            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel1.getCurrentPosition())));
//            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel2.getCurrentPosition())));
//
//
//            if (!leftWheel1.isBusy() && !leftWheel2.isBusy() && !rightWheel1.isBusy() && !rightWheel2.isBusy()) {
//                subStep += 0.5;
//            }
//        }
            //step = 4;


        }

        String filename = "test.csv";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, csvData);
        telemetry.log().add("saved to '%s'", filename);

    }
}
