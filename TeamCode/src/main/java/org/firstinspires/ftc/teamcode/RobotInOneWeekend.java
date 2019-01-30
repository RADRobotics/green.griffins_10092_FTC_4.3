package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.hardware.Sensor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/* Authors: Programming team: Aidan, Chase, Andrew, and Keiko, with sage guidance from greatest programmer Stanley and programming mentor Brian
   Purpose: This is the third test iteration of the Tele-Op program for the RI1W robot, which has all of the basic functions, and encoder telemetry.
   Input: The user runs the program with a phone, then controls the robot with a gamepad connected to said phone
   Output: The robot moves around and stuff based on user input through the gamepads.
   I know you're wondering-when did we actually comment? Of course, it's all thanks to greatest programmer Stanley.
   But seriously all of the comments are mine.
   Also the "greatest programmer" stuff was a joke please don't kill me.
   -Supreme Overlord Stanley (if it wasn't obvious)
*/
/* Dear Supreme Leader Stanley,
   It has come to my attention that you have not included me in the author comments, have not updated them accordingly,
   and not given your proper respects to the supreme chancellor, me, Andrew.
   Notice that I have already edited your comments accordingly and judged your work insufficient.
   I expect you to step up your game, or, you will be promptly terminated and removed...you know what I am referring too.
   Submission is imminent!
   -Supreme Chancellor Andrew (if you couldn't already tell, I am your ruler)
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "RobotInOneWeekend", group = "Prototyping")
public class RobotInOneWeekend extends OpMode {

    //defines the various motors and servos in the robot-the drivetrain motors, arm motors, intake motor, and lock servos
    private DcMotor leftWheel;
    private DcMotor rightWheel;
    private DcMotor leftArm;
    private DcMotor rightArm;
    private DcMotor intake;
    private Servo lock1;
    private Servo lock2;
    private int leftWheelEncoder;
    private int rightWheelEncoder;
    private int leftArmEncoder;
    private int rightArmEncoder;
    //private ColorSensor leftColor;
    //private DistanceSensor rightDistance;

    //sets doubles for set point of arm and nitro value
    private double setpoint = 0;
    private double nitro = 0;
    private double nitro2 = 0;

    //defines constants that are the multipliers applied to nitro-multipliers are based on base speed multipliers later in the program
    static final private double nitroDriveTrainMulti = 0.7;
    static final private double nitroIntakeMulti = 0.5;
    static final private double nitroArmMulti = 0.4;

    //defines a boolean that is used to tell the robot to either lock/unlock the latch/lock servos
    private boolean lock = true;

    @Override
    public void init() {
        //defines/sets motors that make up the drivetrain and arm (moving the arm up/down) and servos that help latch/lock
        leftWheel = hardwareMap.dcMotor.get("leftWheel");
        rightWheel = hardwareMap.dcMotor.get("rightWheel");
        leftArm = hardwareMap.dcMotor.get("leftArm");
        rightArm = hardwareMap.dcMotor.get("rightArm");
        intake = hardwareMap.dcMotor.get("intake");
        lock1 = hardwareMap.servo.get("lock1");
        lock2 = hardwareMap.servo.get("lock2");

        leftWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        leftArmEncoder = 0;
        rightArmEncoder = 0;
        rightWheelEncoder = 0;
        leftWheelEncoder = 0;
        //rightDistance = hardwareMap.get(DistanceSensor.class, "rightDistance");
        //leftColor = hardwareMap.colorSensor.get("leftColor");

        //prepares and resets robot by resetting encoders
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //prepares robot for driving by setting drive motors to run using encoders (encoders currently not used)
        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //prepares robot for arm movement by setting arm motors to go to a certain position-modifiable by user input (the d-pad)
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void loop() {
        //sets nitro variable to right trigger (0.7 because base wheel speed multiplier is 0.3, and 0.7+0.3=1)
        nitro = gamepad1.right_trigger * .7;
        nitro2 = 1-gamepad2.right_trigger*.8;

        leftWheelEncoder = leftWheel.getCurrentPosition();
        rightWheelEncoder = rightWheel.getCurrentPosition();
        leftArmEncoder = leftArm.getCurrentPosition();
        rightArmEncoder = rightArm.getCurrentPosition();
        //sets the boolean variable that locks/unlocks the latch/lock servos
        //buttons used: Game pad 1: a, b, y, x, dpad_up, dpad_down,
        if (gamepad1.a) {
            lock = true;
        }
        if (gamepad1.b) {
            lock = false;
        }

        //locks/unlocks servo positions using the lock boolean
        if (lock) {
            lock1.setPosition(1);
            lock2.setPosition(0);
        } else {
            lock1.setPosition(0);
            lock2.setPosition(1);
        }


        //sets intake speed/power-base speed is 50% to make
        if (gamepad1.y) {
            intake.setPower(.5 + (nitro * nitroIntakeMulti));
        } else if (gamepad1.x) {

            intake.setPower(-.5 - (nitro * nitroIntakeMulti));
        } else {
            intake.setPower(0);
        }

        //changes a variable that represents the point at which the arm is set


        //arrrrrrrrrrrrrrrrrrrm

        setpoint = rightArm.getCurrentPosition()+ gamepad1.left_stick_y*125*(gamepad1.right_trigger+1);



//        if (gamepad1.left_stick_button) {
//            leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            setpoint = 0;
//        }
//        if (gamepad1.right_stick_button) {
//            leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }

        //sets speed at which the arm moves and actually sets the arm's position
        rightArm.setPower(.6 + nitro*.6);
        leftArm.setPower(.6 + nitro*.6);
        rightArm.setTargetPosition((int) setpoint);
        leftArm.setTargetPosition(-(int) setpoint);

        //shows set point of the arm for testing purposes
        telemetry.addData("setpoint", setpoint);

//        //creates a variable that takes the joystick positions and uses them to control drivetrain power
//        double leftWheely = -gamepad2.left_stick_y;
//        double rightWheely = gamepad2.right_stick_y;
//        //double army = -gamepad2.left_stick_y;
//
//        //sets drive train power-default 0.3 multiplier because 30% speed is good for normal play (nitro otherwise)
//        double LeftValue =  (leftWheely*leftWheely - nitro2);
//        if(LeftValue<0){
//            LeftValue=0;
//        }
//        double RightValue =  (rightWheely*rightWheely - nitro2);
//        if(RightValue<0){
//            RightValue=0;
//        }


        leftWheel.setPower((gamepad2.left_stick_y-gamepad2.right_stick_x)*nitro2);
        rightWheel.setPower((gamepad2.left_stick_y+gamepad2.right_stick_x)*nitro2);

     //   leftWheel.setPower((leftWheely/Math.abs(leftWheely))* LeftValue);
       // rightWheel.setPower((rightWheely/Math.abs(rightWheely))* RightValue);
        // leftArm.setPower(-army * 0.1);
        // rightArm.setPower(army * 0.1);

        //       float[] hsvValues = {0F, 0F, 0F};
//        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
//        final double SCALE_FACTOR = 255;
//
//        Color.RGBToHSV((int) (leftColor.red() * SCALE_FACTOR),
//                (int) (leftColor.green() * SCALE_FACTOR),
//                (int) (leftColor.blue() * SCALE_FACTOR),
//                hsvValues);
//        ColorSensor colors = ((ColorSensor) leftColor);
//        //Sends color sensor input values to the phone
//        telemetry.addLine()
//                .addData("H",  hsvValues[0])
//                .addData("S",  hsvValues[1])
//                .addData("V",  hsvValues[2]);
//
//        //shows variable that sets motor power for testing purposes
//        telemetry.addData("", "left: %f ", leftWheely);
//        telemetry.addData("", "right: %f ", rightWheely);
//        telemetry.addData("Color Argb: ", leftColor.argb());
//        telemetry.addData("Color Red: ", leftColor.red());
//        telemetry.addData("Color Green: ", leftColor.green());
//        telemetry.addData("Color Blue: ", leftColor.blue());
//        telemetry.addData("Color Alpha: ", leftColor.alpha());
//        telemetry.addData("Distance(MM): ", rightDistance.getDistance(DistanceUnit.MM));
        telemetry.addLine()
                .addData("leftWheel", leftWheelEncoder)
                .addData("rightWheel", rightWheelEncoder)
                .addData("leftArm", leftArmEncoder)
                .addData("rightArm", rightArmEncoder);
        telemetry.update();


        //telemetry.addData("", "Arm: %f ", army);
        //telemetry.addData("","Arm: %f ", army);

        //actually makes the telemetry show what it should (because telemetry is a useless,lazy creature and needs commands to do anything)
        telemetry.update();


    }
}