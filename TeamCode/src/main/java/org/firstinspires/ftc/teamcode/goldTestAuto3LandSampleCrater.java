package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Rect;
import org.opencv.core.Size;


@Autonomous(name="GoldTestAuto3", group="DogeCV")
@Disabled
public class goldTestAuto3LandSampleCrater extends OpMode {


    boolean unlock = true;
    // Detector object
    private ElapsedTime runtime = new ElapsedTime();

    private GoldDetector detector;

    private DcMotor leftWheel;
    private DcMotor rightWheel;
    private final int TICKS_PER_WHEEL_ROTATION = 1120;

    private DcMotor leftArm;
    private DcMotor rightArm;
    private Servo lock1;
    private Servo lock2;
    double rotation = 0;


    private double subStep;

    private enum state {TURN_TO_WALL, MOVE_TO_WALL, MOVE_AWAY_FROM_WALL, TURN_TO_DEPOT, MOVE_TO_DEPOT}

    double alignSize=40;
    double alignX    = (640 / 2) +0; // Center point in X Pixels
    double alignXMin = alignX - (alignSize / 2); // Min X Pos in pixels
    double alignXMax = alignX +(alignSize / 2); // Max X pos in pixels
    double scale = (640-alignSize)/2;
    double stage = -4;

    boolean aligned = false;

    private void resetDriveEncoders() {
        telemetry.addLine("RESET!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n" +
                "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        leftWheel.setPower(0);
        rightWheel.setPower(0);
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    @Override
    public void init() {
        lock1 = hardwareMap.servo.get("lock1");
        lock2 = hardwareMap.servo.get("lock2");

        leftArm = hardwareMap.dcMotor.get("leftArm");
        rightArm = hardwareMap.dcMotor.get("rightArm");
        leftWheel = hardwareMap.dcMotor.get("leftWheel");
        rightWheel = hardwareMap.dcMotor.get("rightWheel");

        leftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftArm.setDirection(DcMotorSimple.Direction.FORWARD);

        leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        boolean unlock = true;
        subStep = 0;




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
    }

    /*
     * Code to run REPEATEDLY when the driver hits INIT
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("bU","");
        telemetry.update();
        beforeUnlock(500);
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY when the driver hits PLAY
     */
    @Override
    public void loop() {

        telemetry.addLine()
                .addData("stage", stage)
                .addData("subStep", subStep);

        telemetry.addLine()
                .addData("RightWheel: ", rightWheel.getCurrentPosition())
                .addData("LeftWheel: ", leftWheel.getCurrentPosition())
                .addData("TICKS_PER_WHEEL_ROTATION: ", TICKS_PER_WHEEL_ROTATION);
        telemetry.update();

        if(stage==-4) {

            if (runtime.seconds()<2) {
                unlockArm();
                telemetry.addData("unlock","");
            }

            if(runtime.seconds()>2) {
                unfold(1925);
                telemetry.addData("unfold","");
            }
            //thiiiiiiiiiiing
            if(runtime.seconds()>5) {
                stage = -3;
                runtime.reset();
                resetDriveEncoders();
            }

            //telemetry.update();

        }
        if(stage==-3.5){
            telemetry.addLine()
                    .addData("IsLeftBusy", leftArm.isBusy())
                    .addData("IsRightBusy", rightArm.isBusy());
            if (!leftArm.isBusy() && !rightArm.isBusy()) {
                stage=-3;
            }
        }
        if(stage==-3) {
            rotation = .3;
            int targetPosition = (int) (rotation * TICKS_PER_WHEEL_ROTATION);
            //         int delta = targetPosition - Math.abs(rightWheel.getCurrentPosition());
            //sets speed at which the wheels move and actually sets the wheels' position
            rightWheel.setTargetPosition(-targetPosition);
            leftWheel.setTargetPosition(targetPosition-(int)(0.1 * TICKS_PER_WHEEL_ROTATION));
            leftWheel.setPower(0.5);
            rightWheel.setPower(0.5);

//            telemetry.addLine()
//                    .addData("Current Position: ", rightWheel.getCurrentPosition())
//                    .addData("Delta", delta);
            if (!leftWheel.isBusy() && !rightWheel.isBusy() && runtime.seconds()>1) {
                stage=-2.5;
                leftArm.setDirection(DcMotorSimple.Direction.REVERSE);
                resetDriveEncoders();
                runtime.reset();
            }
        }
        if(stage==-2.5){
            int targetPosition = (int) (.4 * TICKS_PER_WHEEL_ROTATION);
            rightWheel.setTargetPosition(targetPosition);
            leftWheel.setTargetPosition(targetPosition);
            leftWheel.setPower(0.5);
            rightWheel.setPower(0.5);

            if (!leftWheel.isBusy() && !rightWheel.isBusy() && runtime.seconds()>1) {
                stage=-2.6;
                leftArm.setDirection(DcMotorSimple.Direction.REVERSE);
                resetDriveEncoders();
                runtime.reset();
            }

        }
        if(stage==-2.6){

            int targetPosition = (int) (.3 * TICKS_PER_WHEEL_ROTATION);
            rightWheel.setTargetPosition(targetPosition);
            leftWheel.setTargetPosition(-targetPosition);
            leftWheel.setPower(0.5);
            rightWheel.setPower(0.5);

            if (!leftWheel.isBusy() && !rightWheel.isBusy() && runtime.seconds()>1) {
                stage=-2.6;
                leftArm.setDirection(DcMotorSimple.Direction.REVERSE);
                resetDriveEncoders();
                runtime.reset();
            }
        }
        // leftArm.setDirection(DcMotorSimple.Direction.REVERSE);
        if(stage==-2) {
            rightArm.setTargetPosition((int) (.105 * TICKS_PER_WHEEL_ROTATION * 8));
            leftArm.setTargetPosition((int) (.105 * TICKS_PER_WHEEL_ROTATION * 8));
            rightArm.setPower(.6);
            leftArm.setPower(.6);
            if(runtime.seconds()>4){
                stage=0;
            }
        }

//        telemetry.addData("IsAligned" , detector.getAligned()); // Is the bot aligned with the gold mineral?
//        telemetry.addData("X Pos" , detector.getXPosition()); // Gold X position.

        //Point screenpos = detector.getScreenPosition();
        if(stage==0) {
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
            if(!(xPos>0)){
                rightWheel.setPower(0);
                leftWheel.setPower(0);
                telemetry.addLine("not detected");
            }
            else if (xPos > alignXMax) {
                double power = ((xPos - alignXMax) / scale) * .3 + .4;
                rightWheel.setPower(power);
                leftWheel.setPower(-power);
                telemetry.addData("powL: ", power);
                telemetry.addLine("turning left");
                runtime.reset();
            } else if (xPos < alignXMin) {
                double power = ((alignXMin - xPos) / scale) * .3 + .4;
                rightWheel.setPower(-power);
                leftWheel.setPower(power);
                telemetry.addData("powR: ", power);
                telemetry.addLine("turning right");
                runtime.reset();
            } else {
                rightWheel.setPower(0);
                leftWheel.setPower(0);
                telemetry.addLine("found");
                telemetry.addData("secks: ", runtime.seconds());
                if(runtime.seconds()>.1){
                    runtime.reset();
                    stage++;
                    resetDriveEncoders();
                }
            }


        }
        else if (stage==1){
            rightWheel.setTargetPosition(-3*TICKS_PER_WHEEL_ROTATION);
            leftWheel.setTargetPosition(-3*TICKS_PER_WHEEL_ROTATION);
            leftWheel.setPower(.5);
            rightWheel.setPower(.5);
            if(runtime.seconds()>5){
                rightWheel.setPower(0);
                leftWheel.setPower(0);
            }
        stage = 2;
        }
        if ((subStep * 2) % 2 == 1) {
            resetDriveEncoders();
            subStep += 0.5;
        } else if (subStep == goldTestAuto3LandSampleCrater.state.TURN_TO_WALL.ordinal()) {
            // turns the robot 90 degrees counter clockwise
            double target = .75;

            leftWheel.setPower(0.5);
            rightWheel.setPower(0.5);

            leftWheel.setTargetPosition((int) (-target * TICKS_PER_WHEEL_ROTATION));
            rightWheel.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));

            telemetry.addData("delta", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel.getCurrentPosition())));

            if (!leftWheel.isBusy() && !rightWheel.isBusy()) {
                subStep += 0.5;
            }
        }
//
        else if (subStep == goldTestAuto3LandSampleCrater.state.MOVE_TO_WALL.ordinal()) {
            // moves the robot forward up against the wall

            double target = 3;

            leftWheel.setPower(0.5);
            rightWheel.setPower(0.5);

            leftWheel.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
            rightWheel.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));

            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel.getCurrentPosition())));

            if (!leftWheel.isBusy() && !rightWheel.isBusy()) {
                subStep += 0.5;
            }
        } else if (subStep == goldTestAuto3LandSampleCrater.state.MOVE_AWAY_FROM_WALL.ordinal()) {
            // moves the robot backward away from the wall
            double target = -1;

            leftWheel.setPower(0.5);
            rightWheel.setPower(0.5);

            leftWheel.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
            rightWheel.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));

            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel.getCurrentPosition())));

            if (!leftWheel.isBusy() && !rightWheel.isBusy()) {
                subStep += 0.5;

            }
        } else if (subStep == goldTestAuto3LandSampleCrater.state.TURN_TO_DEPOT.ordinal()) {
            // turns the robot counter-clockwise to line up with the depot
            double target = 1;

            leftWheel.setPower(0.5);
            rightWheel.setPower(0.5);

            leftWheel.setTargetPosition((int) (-target * TICKS_PER_WHEEL_ROTATION));
            rightWheel.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));

            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel.getCurrentPosition())));

            if (!leftWheel.isBusy() && !rightWheel.isBusy()) {
                subStep += 0.5;
            }
        } else if (subStep == goldTestAuto3LandSampleCrater.state.MOVE_TO_DEPOT.ordinal()) {
            // moves the robot forward up to the depot
            double target = 5;

            leftWheel.setPower(0.5);
            rightWheel.setPower(0.5);

            leftWheel.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
            rightWheel.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));

            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheel.getCurrentPosition())));

            if (!leftWheel.isBusy() && !rightWheel.isBusy()) {
                subStep += 0.5;
            }
        }
    }





    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        detector.disable();
    }
    private void beforeUnlock(int point) {

        leftArm.setPower(-1.0);
        rightArm.setPower(1.0);
        //double aPoint=0;
        long startTime = System.currentTimeMillis();
        long currentTime = System.currentTimeMillis();
        long deltaTime = currentTime - startTime;

        while (deltaTime < 1000) {
            //aPoint -= 3;
            rightArm.setTargetPosition(-point);
            leftArm.setTargetPosition(point);

            currentTime = System.currentTimeMillis();
            deltaTime = currentTime - startTime;

            //sets speed at which the arm moves and actually sets the arm's position
            //rightArm.setTargetPosition((int) aPoint);
            //leftArm.setTargetPosition(-(int) aPoint);
        }

    }

    private void unlockArm() {

        lock1.setPosition(1);
        lock2.setPosition(0);
    }

//    private void lockArm(){
//
//        lock1.setPosition(1);
//        lock2.setPosition(0);
//    }

    private void unfold(double finalSetpoint) {

        //double setpoint = 0;

        rightArm.setPower(0.65);
        leftArm.setPower(0.65);

        //while (setpoint < finalSetpoint) {
        //setpoint += 3;

        //sets speed at which the arm moves and actually sets the arm's position
        rightArm.setTargetPosition((int) finalSetpoint);
        leftArm.setTargetPosition(-(int) finalSetpoint);
        //}
    }

    private void rotate() {


         telemetry.update();
    }

}






