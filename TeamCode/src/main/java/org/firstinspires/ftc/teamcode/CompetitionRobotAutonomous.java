package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

//dab
@Autonomous(name = "CompetitionRobotAuto", group = "Prototyping")
@Disabled
public class CompetitionRobotAutonomous extends LinearOpMode {

    //defines the various motors, Sensors, and variables in the robot
    private Servo rightLock;
    private Servo leftLock;

    private DcMotor leftWheelFront;
    private DcMotor leftWheelBack;
    private DcMotor rightWheelFront;
    private DcMotor rightWheelBack;
    private DcMotor leftArm;
    private DcMotor rightArm;
    private ColorSensor leftColor;
    private DistanceSensor rightDistance;
    private int count;
    private boolean sample_ready;
    private boolean lander_unlatched;
    private int step;
    private final double TICKS_PER_WHEEL_ROTATION = 1120;
    private double subStep;

    private enum state {TURN_TO_WALL, MOVE_TO_WALL, MOVE_AWAY_FROM_WALL, TURN_TO_DEPOT, MOVE_TO_DEPOT}



    public void runOpMode() throws InterruptedException {
        rightLock = hardwareMap.servo.get("rightLock");
        leftLock = hardwareMap.servo.get("leftLock");
        //defines/sets motors that make up the drivetrain and arm (moving the arm up/down)
        leftWheelFront = hardwareMap.dcMotor.get("leftWheelFront");
        leftWheelBack = hardwareMap.dcMotor.get("leftWheelBack");
        rightWheelFront = hardwareMap.dcMotor.get("rightWheelFront");
        rightWheelBack = hardwareMap.dcMotor.get("rightWheelBack");
        leftArm = hardwareMap.dcMotor.get("leftArm");
        rightArm = hardwareMap.dcMotor.get("rightArm");
        rightDistance = hardwareMap.get(DistanceSensor.class, "rightDistance");
        leftColor = hardwareMap.colorSensor.get("leftColor");
        rightWheelFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightWheelBack.setDirection(DcMotorSimple.Direction.REVERSE);
        count = 21;
        step = 0;
        subStep = 0;

        sample_ready = false;
        lander_unlatched = false;

        //prepares and resets robot by resetting encoders
        resetDriveEncoders();

        //prepares robot for arm movement by setting arm motors to go to a certain position-modifiable by user input (the d-pad)
//        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //HSV value array

        //Sends color sensor input values to the phone
        waitForStart();

        while (opModeIsActive()) {
            telemetry.update();


            telemetry.addLine()
                    .addData("step", step)
                    .addData("subStep", subStep)
                    .addData("count", count);
            telemetry.addLine()
                    .addData("RightWheelFront: ", rightWheelFront.getCurrentPosition())
                    .addData("RightWheelBack: ", rightWheelBack.getCurrentPosition())
                    .addData("LeftWheelFront: ", leftWheelFront.getCurrentPosition())
                    .addData("LeftWheelBack: ", leftWheelBack.getCurrentPosition())
                    .addData("TICKS_PER_WHEEL_ROTATION: ", TICKS_PER_WHEEL_ROTATION);


            if (step == 0) {
                leftLock.setPosition(0.6);
                rightLock.setPosition(.4);
                leftArm.setPower(.5);
                leftArm.setPower(.5);

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
        leftWheelFront.setPower(0);
        leftWheelBack.setPower(0);
        rightWheelFront.setPower(0);
        rightWheelBack.setPower(0);
        leftWheelFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWheelBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheelFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheelBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWheelFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftWheelBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheelFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheelBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    private void moveToDepot() throws InterruptedException {

        if ((subStep * 2) % 2 == 1) {
            resetDriveEncoders();
            subStep += 0.5;
        } else if (subStep == state.TURN_TO_WALL.ordinal()) {
            // turns the robot 90 degrees counter clockwise
            double target = .75;

            leftWheelFront.setPower(0.5);
            leftWheelBack.setPower(0.5);
            rightWheelFront.setPower(0.5);
            rightWheelBack.setPower(0.5);

            leftWheelFront.setTargetPosition((int) (-target * TICKS_PER_WHEEL_ROTATION));
            leftWheelBack.setTargetPosition((int) (-target * TICKS_PER_WHEEL_ROTATION));
            rightWheelFront.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
            rightWheelBack.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));

            telemetry.addData("delta", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheelFront.getCurrentPosition())));
            telemetry.addData("delta", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheelBack.getCurrentPosition())));

            if (!leftWheelFront.isBusy() && !leftWheelBack.isBusy() && !rightWheelFront.isBusy() && !rightWheelBack.isBusy()) {
                subStep += 0.5;
            }
        }
//
        else if (subStep == state.MOVE_TO_WALL.ordinal()) {
            // moves the robot forward up against the wall

            double target = 3;

            leftWheelFront.setPower(0.5);
            leftWheelBack.setPower(0.5);
            rightWheelFront.setPower(0.5);
            rightWheelBack.setPower(0.5);

            leftWheelFront.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
            leftWheelBack.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
            rightWheelFront.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
            rightWheelBack.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));

            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheelFront.getCurrentPosition())));
            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheelBack.getCurrentPosition())));


            if (!leftWheelFront.isBusy() && !leftWheelBack.isBusy() && !rightWheelFront.isBusy() && !rightWheelBack.isBusy()) {
                subStep += 0.5;
            }
        } else if (subStep == state.MOVE_AWAY_FROM_WALL.ordinal()) {
            // moves the robot backward away from the wall
            double target = -1;

            leftWheelFront.setPower(0.5);
            leftWheelBack.setPower(0.5);
            rightWheelFront.setPower(0.5);
            rightWheelBack.setPower(0.5);

            leftWheelFront.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
            leftWheelBack.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
            rightWheelFront.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
            rightWheelBack.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));

            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheelFront.getCurrentPosition())));
            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheelBack.getCurrentPosition())));


            if (!leftWheelFront.isBusy() && !leftWheelBack.isBusy() && !rightWheelFront.isBusy() && !leftWheelBack.isBusy()) {
                subStep += 0.5;

            }
        } else if (subStep == state.TURN_TO_DEPOT.ordinal()) {
            // turns the robot counter-clockwise to line up with the depot
            double target = 1;

            leftWheelFront.setPower(0.5);
            leftWheelBack.setPower(0.5);
            rightWheelFront.setPower(0.5);
            rightWheelBack.setPower(0.5);

            leftWheelFront.setTargetPosition((int) (-target * TICKS_PER_WHEEL_ROTATION));
            leftWheelBack.setTargetPosition((int) (-target * TICKS_PER_WHEEL_ROTATION));
            rightWheelFront.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
            rightWheelBack.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));

            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheelFront.getCurrentPosition())));
            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheelBack.getCurrentPosition())));


            if (!leftWheelFront.isBusy() && !leftWheelBack.isBusy() && !rightWheelFront.isBusy() && !rightWheelBack.isBusy()) {
                subStep += 0.5;
            }
        } else if (subStep == state.MOVE_TO_DEPOT.ordinal()) {
            // moves the robot forward up to the depot
            double target = 5;

            leftWheelFront.setPower(0.5);
            leftWheelBack.setPower(0.5);
            rightWheelFront.setPower(0.5);
            rightWheelBack.setPower(0.5);

            leftWheelFront.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
            leftWheelBack.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
            rightWheelFront.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));
            rightWheelBack.setTargetPosition((int) (target * TICKS_PER_WHEEL_ROTATION));

            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheelFront.getCurrentPosition())));
            telemetry.addData("delta2 ", (target * TICKS_PER_WHEEL_ROTATION - Math.abs(leftWheelBack.getCurrentPosition())));


            if (!leftWheelFront.isBusy() && !leftWheelBack.isBusy() && !rightWheelFront.isBusy() && !rightWheelBack.isBusy()) {
                subStep += 0.5;
            }
        }
        //step = 4;
    }
}
