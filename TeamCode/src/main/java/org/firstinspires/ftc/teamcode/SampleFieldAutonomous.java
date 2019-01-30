package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//dab
@Autonomous(name = "Sample Field Autonomous", group = "Prototyping")
public class SampleFieldAutonomous extends LinearOpMode {

    //defines the various motors, Sensors, and variables in the robot
    private DcMotor leftWheel;
    private DcMotor rightWheel;
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

        //defines/sets motors that make up the drivetrain and arm (moving the arm up/down)
        leftWheel = hardwareMap.dcMotor.get("leftWheel");
        rightWheel = hardwareMap.dcMotor.get("rightWheel");
        leftArm = hardwareMap.dcMotor.get("leftArm");
        rightArm = hardwareMap.dcMotor.get("rightArm");
        rightDistance = hardwareMap.get(DistanceSensor.class, "rightDistance");
        leftColor = hardwareMap.colorSensor.get("leftColor");
        rightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        count = 21;
        step = 3;
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
                    .addData("RightWheel: ", rightWheel.getCurrentPosition())
                    .addData("LeftWheel: ", leftWheel.getCurrentPosition())
                    .addData("TICKS_PER_WHEEL_ROTATION: ", TICKS_PER_WHEEL_ROTATION);


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
    }

    private void resetDriveEncoders()  {
        telemetry.addLine("RESET!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n" +
                "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        leftWheel.setPower(0);
        rightWheel.setPower(0);
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    private void moveToDepot() throws InterruptedException {

        if ((subStep * 2) % 2 == 1) {
            resetDriveEncoders();
            subStep += 0.5;
        } else if (subStep == state.TURN_TO_WALL.ordinal()) {
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
        else if (subStep == state.MOVE_TO_WALL.ordinal()) {
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
        } else if (subStep == state.MOVE_AWAY_FROM_WALL.ordinal()) {
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
        } else if (subStep == state.TURN_TO_DEPOT.ordinal()) {
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
        } else if (subStep == state.MOVE_TO_DEPOT.ordinal()) {
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
        //step = 4;
    }
}
