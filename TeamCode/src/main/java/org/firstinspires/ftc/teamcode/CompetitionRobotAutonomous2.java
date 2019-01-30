package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Rect;
import org.opencv.core.Size;

//dab
@Autonomous(name = "CompetitionRobotAuto2", group = "Prototyping")

public class CompetitionRobotAutonomous2 extends LinearOpMode {

    //defines the various motors, Sensors, and variables in the robot
    private ElapsedTime runtime = new ElapsedTime();

    private GoldDetector detector;
    private Servo rightLock;
    private Servo leftLock;

    private DcMotor leftWheel1;
    private DcMotor leftWheel2;
    private DcMotor rightWheel1;
    private DcMotor rightWheel2;
    private DcMotor leftArm;
    private DcMotor rightArm;
    private DcMotor armExtendRight;
    private DcMotor armExtendLeft;
    int subStep=0;
    private int step;
    private final double TICKS_PER_WHEEL_ROTATION = 1120;


    private enum state {TURN_TO_WALL, MOVE_TO_WALL, MOVE_AWAY_FROM_WALL, TURN_TO_DEPOT, MOVE_TO_DEPOT}

    double alignSize=40;
    double alignX    = 380;//  (640 / 2) +0; // Center point in X Pixels
    double alignXMin = alignX - (alignSize / 2); // Min X Pos in pixels
    double alignXMax = alignX +(alignSize / 2); // Max X pos in pixels
    double scale = (640)/2;//-alignSize
    double stage = -4;

    boolean aligned = false;
    boolean targeted = false;
    int arm = 0;

    public void runOpMode() throws InterruptedException {
        rightLock = hardwareMap.servo.get("rightLock");
        leftLock = hardwareMap.servo.get("leftLock");
        //defines/sets motors that make up the drivetrain and arm (moving the arm up/down)
        leftWheel1 = hardwareMap.dcMotor.get("leftWheel");
        leftWheel2 = hardwareMap.dcMotor.get("leftWheel2");
        rightWheel1 = hardwareMap.dcMotor.get("rightWheel");
        rightWheel2 = hardwareMap.dcMotor.get("rightWheel2");
        leftArm = hardwareMap.dcMotor.get("leftArm");
        rightArm = hardwareMap.dcMotor.get("rightArm");
        armExtendRight = hardwareMap.dcMotor.get("armExtend");
        armExtendLeft = hardwareMap.dcMotor.get("armExtend2");
        armExtendLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtendLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        rightWheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        step = -1;

        telemetry.addData("Status", "DogeCV 2018.0 - Gold Detector test");
        // Setup detector
        detector = new GoldDetector(); // Create detector
        detector.setAdjustedSize(new Size(480, 270)); // Set detector size
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize detector with app context and camera
        detector.useDefaults(); // Set default detector settings
        // Optional tuning

        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;
        detector.enable(); // Start detector












        //prepares and resets robot by resetting encoders
        //resetDriveEncoders();

        //prepares robot for arm movement by setting arm motors to go to a certain position-modifiable by user input (the d-pad)
//        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //HSV value array

        //Sends color sensor input values to the phone
        waitForStart();
runtime.reset();
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftWheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftWheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {


            telemetry.update();


            telemetry.addLine()
                    .addData("step", step);
            telemetry.addLine()
                    .addData("rightWheel1: ", leftWheel1.getCurrentPosition())
                    .addData("rightWheel1: ", rightWheel1.getCurrentPosition());
                    //.addData("TICKS_PER_WHEEL_ROTATION: ", TICKS_PER_WHEEL_ROTATION);
telemetry.addData("arm::::::",arm);
telemetry.addData("armExtend",armExtendLeft.getCurrentPosition());
telemetry.addData("armExtend2",armExtendRight.getCurrentPosition());

arm = leftArm.getCurrentPosition()-1300;//695
            if(step==-1){
                leftLock.setPosition(0.37);
                rightLock.setPosition(.32);

                    rightArm.setPower(.5);
                    leftArm.setPower(-.5);

                if(runtime.seconds()>2 || arm >-1100){
                    step=0;
                    runtime.reset();
                }
            }
            if (step == 0) {
                leftLock.setPosition(0.37);
                rightLock.setPosition(.32);

                if(arm>-4250) {
                    //leftArm.setPower(.3);

                    rightArm.setPower(-.15);
                    leftArm.setPower(.15);

                }
                else if (arm>-4900){
                    //leftArm.setPower(.65);

                    rightArm.setPower(-.3);
                    leftArm.setPower(.3);

                    //rightArm.setPower(-.65);
                    if(rightWheel1.getCurrentPosition()>-200) {
                        leftWheel2.setPower(-.4);
                        leftWheel1.setPower(-.4);
                        rightWheel2.setPower(-.4);
                        rightWheel1.setPower(-.4);
                    }
                    else{
                        leftWheel2.setPower(0);
                        leftWheel1.setPower(0);
                        rightWheel2.setPower(0);
                        rightWheel1.setPower(0);
                    }
                }
                else{
                    leftWheel2.setPower(0);
                    leftWheel1.setPower(0);
                    rightWheel2.setPower(0);
                    rightWheel1.setPower(0);
                   // leftArm.setPower(0);
                    rightArm.setPower(0);
                    step=1;
                    runtime.reset();
                }

            }
            if(step==1){
                //leftArm.setPower(-.55);
                 //rightArm.setPower(.55);

                rightArm.setPower(.2);
                leftArm.setPower(-.2);

                if(runtime.seconds()>2.5 || leftArm.getCurrentPosition()>-1000){
                    leftArm.setPower(0);
                    rightArm.setPower(0);
                    runtime.reset();
                    step=2;
                    rightWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightWheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    leftWheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            }
            if(step==2) {
                Rect bestRect = detector.getFoundRect();

                double xPos = bestRect.x + (bestRect.width / 2);

                if (xPos < alignXMax && xPos > alignXMin) {
                    aligned = true;
                } else {
                    aligned = false;
                }
                telemetry.addData("aligned ", aligned);

                telemetry.addData("xpos ", xPos);
                telemetry.addData("amax ", alignXMax);
                telemetry.addData("amin ", alignXMin);
                if((xPos>0) && !targeted){

                    if(xPos>alignX-70 && xPos<alignX+70){
                        alignSize=140;
                    }
                    if(xPos<150){
                        alignX=alignX-25;
                    }
                    if(xPos<(640-150)){
                        alignX=alignX+25;
                    }
                    targeted=true;
                  //  double alignXMin = alignX - (alignSize / 2); // Min X Pos in pixels
                    //double alignXMax = alignX +(alignSize / 2); // Max X pos in pixels
                }
                if(!(xPos>0)){
                    if(runtime.seconds()<4) {
                        rightWheel1.setPower(0);
                        rightWheel2.setPower(0);
                        leftWheel1.setPower(0);
                        leftWheel2.setPower(0);
                    }
                    telemetry.addLine("not detected");

                    if(runtime.seconds()>4 && runtime.seconds()<5){
                        if(rightWheel1.getCurrentPosition()>-150) {
                            rightWheel1.setPower(-.4);
                            rightWheel2.setPower(-.4);
                            leftWheel1.setPower(.4);
                            leftWheel2.setPower(.4);
                        }
                        else{
                            rightWheel1.setPower(0);
                            rightWheel2.setPower(0);
                            leftWheel1.setPower(0);
                            leftWheel2.setPower(0);
                        }
                    }
                    if(runtime.seconds()>5 && runtime.seconds()<7){
                        if(rightWheel1.getCurrentPosition()<150) {
                            rightWheel1.setPower(.4);
                            rightWheel2.setPower(.4);
                            leftWheel1.setPower(-.4);
                            leftWheel2.setPower(-.4);
                        }
                        else{
                            rightWheel1.setPower(0);
                            rightWheel2.setPower(0);
                            leftWheel1.setPower(0);
                            leftWheel2.setPower(0);
                        }
                    }
                    if(runtime.seconds()>7){
                        if(rightWheel1.getCurrentPosition()>1) {
                            rightWheel1.setPower(-.4);
                            rightWheel2.setPower(-.4);
                            leftWheel1.setPower(.4);
                            leftWheel2.setPower(.4);
                        }
                        else{
                            runtime.reset();
                            step++;
                            rightWheel1.setPower(0);
                            rightWheel2.setPower(0);
                            leftWheel1.setPower(0);
                            leftWheel2.setPower(0);
                        }

                    }
                }
                else if (xPos > alignXMax) {
                    double power = ((xPos - alignXMax) / scale) * .12 + .11;
                    rightWheel1.setPower(-power);
                    rightWheel2.setPower(-power);
                    leftWheel1.setPower(power);
                    leftWheel2.setPower(power);
                    telemetry.addData("powL: ", power);
                    telemetry.addLine("turning left");
                    runtime.reset();
                } else if (xPos < alignXMin) {
                    double power = ((alignXMin - xPos) / scale) * .12 + .11 ;
                    rightWheel1.setPower(power);
                    rightWheel2.setPower(power);
                    leftWheel1.setPower(-power);
                    leftWheel2.setPower(-power);
                    telemetry.addData("powR: ", power);
                    telemetry.addLine("turning right");
                    runtime.reset();
                } else {
                    rightWheel1.setPower(0);
                    rightWheel2.setPower(0);
                    leftWheel1.setPower(0);
                    leftWheel2.setPower(0);
                    telemetry.addLine("found");
                    telemetry.addData("secks: ", runtime.seconds());
                    if(runtime.seconds()>.3){
                        runtime.reset();
                        step++;
                        //resetDriveEncoders();
                    }
                }


            }
            else if (step==3){
                if(rightWheel1.getCurrentPosition()<1550) {
                    leftWheel2.setPower(.4);
                    leftWheel1.setPower(.4);
                    rightWheel2.setPower(.4);
                    rightWheel1.setPower(.4);
                }
                else {
                    rightWheel1.setPower(0);
                    rightWheel2.setPower(0);
                    leftWheel1.setPower(0);
                    leftWheel2.setPower(0);
                    step=4;
                }
                //stage = 2;
            }
            else if(step==4){

                if(armExtendLeft.getCurrentPosition()<400){
                    armExtendLeft.setPower(-.3);
                    armExtendRight.setPower(.3);
                }
                else{
                    armExtendLeft.setPower(0);
                    armExtendRight.setPower(0);
                }
            }

    /*
//sets the step of where we are in the process to one
            // step = 1;
//enables the if statement for lander_unlatched
            lander_unlatched = true;
//moves the robot from the lander to where the samples are
            if (step == 1) {
//                leftWheel.setPower(0.7);
//                rightWheel.setPower(0.5);
//                leftWheel.setTargetPosition(1621);
//                rightWheel.setTargetPosition(820);
                step = 2;
            }
            if (step == 2) {
//                if (hsvValues[0] > 50) {
//                    leftWheel.setPower(-0.7);
//                    rightWheel.setPower(-0.5);
//                    leftWheel.setTargetPosition(1221);
                step = 3;
            }
//            if step 2 is finished, makes the robot turn toward the wall, then drives against the wall.
            if (step == 3) {
                moveToDepot();
            }
            //1150 Target Units == about 1 foot, 96 Units/inch
//sets the wheel power to 0 to limit movement
//                if (rightWheel.getCurrentPosition() == -rotation) {
            // rightWheel.setPower(0);
            //leftWheel.setPower(0);
//                }
//               if (leftWheel.getCurrentPosition() == 100) {
//                    leftWheel.setPower(0);
//                    rightWheel.setPower(0);
//                }

            sample_ready = true;

//        if (sample_ready) {
//
        }

        */
        }
    }

    private void resetDriveEncoders() {
        telemetry.addLine("RESET!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n" +
                "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        leftWheel1.setPower(0);
        leftWheel2.setPower(0);
        rightWheel1.setPower(0);
        rightWheel2.setPower(0);
        leftWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWheel1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftWheel2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

    private void moveToDepot() throws InterruptedException {

        if ((subStep * 2) % 2 == 1) {
            resetDriveEncoders();
            subStep += 0.5;
        } else if (subStep == state.TURN_TO_WALL.ordinal()) {
            // turns the robot 90 degrees counter clockwise
            double target = .75;

            leftWheel1.setPower(0.5);
            leftWheel2.setPower(0.5);
            rightWheel1.setPower(0.5);
            rightWheel2.setPower(0.5);

            leftWheel1.setTargetPosition((int) (-target * TICKS_PER_WHEEL_ROTATION));
            leftWheel2.setTargetPosition((int) (-target * TICKS_PER_WHEEL_ROTATION));
            rightWheel1.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
            rightWheel2.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));

            telemetry.addData("delta", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel1.getCurrentPosition())));
            telemetry.addData("delta", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel2.getCurrentPosition())));

            if (!leftWheel1.isBusy() && !leftWheel2.isBusy() && !rightWheel1.isBusy() && !rightWheel2.isBusy()) {
                subStep += 0.5;
            }
        }
//
        else if (subStep == state.MOVE_TO_WALL.ordinal()) {
            // moves the robot forward up against the wall

            double target = 3;

            leftWheel1.setPower(0.5);
            leftWheel2.setPower(0.5);
            rightWheel1.setPower(0.5);
            rightWheel2.setPower(0.5);

            leftWheel1.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
            leftWheel2.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
            rightWheel1.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
            rightWheel2.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));

            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel1.getCurrentPosition())));
            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel2.getCurrentPosition())));


            if (!leftWheel1.isBusy() && !leftWheel2.isBusy() && !rightWheel1.isBusy() && !rightWheel2.isBusy()) {
                subStep += 0.5;
            }
        } else if (subStep == state.MOVE_AWAY_FROM_WALL.ordinal()) {
            // moves the robot backward away from the wall
            double target = -1;

            leftWheel1.setPower(0.5);
            leftWheel2.setPower(0.5);
            rightWheel1.setPower(0.5);
            rightWheel2.setPower(0.5);

            leftWheel1.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
            leftWheel2.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
            rightWheel1.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
            rightWheel2.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));

            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel1.getCurrentPosition())));
            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel2.getCurrentPosition())));


            if (!leftWheel1.isBusy() && !leftWheel2.isBusy() && !rightWheel1.isBusy() && !leftWheel2.isBusy()) {
                subStep += 0.5;

            }
        } else if (subStep == state.TURN_TO_DEPOT.ordinal()) {
            // turns the robot counter-clockwise to line up with the depot
            double target = 1;

            leftWheel1.setPower(0.5);
            leftWheel2.setPower(0.5);
            rightWheel1.setPower(0.5);
            rightWheel2.setPower(0.5);

            leftWheel1.setTargetPosition((int) (-target * TICKS_PER_WHEEL_ROTATION));
            leftWheel2.setTargetPosition((int) (-target * TICKS_PER_WHEEL_ROTATION));
            rightWheel1.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
            rightWheel2.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));

            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel1.getCurrentPosition())));
            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel2.getCurrentPosition())));


            if (!leftWheel1.isBusy() && !leftWheel2.isBusy() && !rightWheel1.isBusy() && !rightWheel2.isBusy()) {
                subStep += 0.5;
            }
        } else if (subStep == state.MOVE_TO_DEPOT.ordinal()) {
            // moves the robot forward up to the depot
            double target = 5;

            leftWheel1.setPower(0.5);
            leftWheel2.setPower(0.5);
            rightWheel1.setPower(0.5);
            rightWheel2.setPower(0.5);

            leftWheel1.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
            leftWheel2.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
            rightWheel1.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
            rightWheel2.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));

            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel1.getCurrentPosition())));
            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel2.getCurrentPosition())));


            if (!leftWheel1.isBusy() && !leftWheel2.isBusy() && !rightWheel1.isBusy() && !rightWheel2.isBusy()) {
                subStep += 0.5;
            }
        }
    }
}
