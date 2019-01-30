package org.firstinspires.ftc.teamcode;

import android.media.AudioManager;
import android.media.SoundPool;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "CompetitionRobotTeleOp", group = "Competition")

public class CompetitionRobotTeleOp extends OpMode {
    private DcMotor leftWheelFront;
    //front top left wheel
    private DcMotor leftWheelBack;
    //back bottom left wheel
    private DcMotor rightWheelFront;
    //front top right wheel
    private DcMotor rightWheelBack;
    //back bottom right wheel
    private DcMotor rightArm;
    private DcMotor leftArm;
    private DcMotor armExtendRight;
    //arm extend right
    private DcMotor armExtendLeft;
    //arm extend left
    private int armExtendRightEncoder;
    private int armExtendLeftEncoder;
    private int leftArmEncoder;
    private int rightArmEncoder;
    private double nitro1;
    private double nitro2;
    private double nitro3;

    boolean cos = false;
    boolean press = false;
    private Servo rightLock;
    private Servo leftLock;
    private Servo intake;
    private Servo intake2;

    private double setpoint = 0;

    private boolean isLocked = false;

    private int armSetPoint = 0;
    private int armExtendSetPoint = 0;
    private boolean armPIDActive = false;
    double theta;

    public SoundPool mySound;
    public int beepID;
    int streamID;
    boolean play = false;
    @Override
    public void init() {
        mySound= new SoundPool(1,AudioManager.STREAM_MUSIC,0);
        beepID = mySound.load(hardwareMap.appContext, R.raw.mario,1);

        rightWheelFront = hardwareMap.dcMotor.get("rightWheel");
        leftWheelFront = hardwareMap.dcMotor.get("leftWheel");
        leftWheelBack = hardwareMap.dcMotor.get("leftWheel2");

        rightWheelBack = hardwareMap.dcMotor.get("rightWheel2");
        rightArm = hardwareMap.dcMotor.get("rightArm");
        leftArm = hardwareMap.dcMotor.get("leftArm");
        armExtendRight = hardwareMap.dcMotor.get("armExtend");
        armExtendLeft = hardwareMap.dcMotor.get("armExtend2");
        rightLock = hardwareMap.servo.get("rightLock");
        leftLock = hardwareMap.servo.get("leftLock");
        intake = hardwareMap.servo.get("intake");
        intake2 = hardwareMap.servo.get("intake2");

        rightLock.setPosition(.6);
        leftLock.setPosition(.4);

//        leftWheelFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftWheelBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightWheelFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightWheelBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
       // armExtendLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armExtendRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        leftWheelFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftWheelBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightWheelFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightWheelBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
        armExtendLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtendRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    @Override
    public void loop() {
        if((gamepad1.right_trigger>.1 || gamepad2.right_trigger>.1) && play==false){
           streamID= mySound.play(beepID,1,1,1,-1,1);
            play = true;
        }
        else if(gamepad1.right_trigger>.1 || gamepad2.right_trigger>.1){

        }
        else{
            play = false;
            mySound.stop(streamID);
        }
        nitro1 = 1 - (gamepad1.right_trigger * .4);
        nitro2 = 1 + (gamepad2.right_trigger);
        nitro3 = 1 - (gamepad2.left_trigger * .8);


        //drive motors

            rightWheelFront.setPower((-gamepad1.left_stick_y + gamepad1.right_stick_x) * nitro1);
            rightWheelBack.setPower((-gamepad1.left_stick_y + gamepad1.right_stick_x) * nitro1);
            leftWheelBack.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x) * nitro1);
            leftWheelFront.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x) * nitro1);

        //arm extension motors
        if (!armPIDActive) {
            armExtendRight.setPower(.5 * (-gamepad2.right_stick_y) * nitro2 * (1 - (gamepad2.left_trigger * .8)));
            armExtendLeft.setPower(.5 * (gamepad2.right_stick_y) * nitro2 * (1 - (gamepad2.left_trigger * .8)));
        }
        //arm motors/lock servos
//        setpoint = /*rightArm.getCurrentPosition()+*/ gamepad2.left_stick_y*125*(nitro2);

        if (gamepad2.x) {
            isLocked = true;
        }
        if (gamepad2.a) {
            isLocked = false;
        }


        if (gamepad2.left_bumper) {
            if (gamepad2.b) {
                intake.setPosition(0.25);
            } else {
                intake.setPosition(.75);
            }
            telemetry.addData("test2", gamepad1.left_bumper);
        } else {
            intake.setPosition(0);
        }

        if (gamepad2.right_bumper) {
            if (gamepad2.b) {
                intake2.setPosition(0.25);
            } else {
                intake2.setPosition(0.75);
            }
            telemetry.addData("test", gamepad1.right_bumper);
        } else {
            intake2.setPosition(0);
        }

        if (isLocked) {
            if (gamepad2.left_stick_y < 0) {
                rightArm.setPower(.5 * -gamepad2.left_stick_y * nitro2 * (1 - (gamepad2.left_trigger * .8)));
                leftArm.setPower(.5 * gamepad2.left_stick_y * nitro2 * (1 - (gamepad2.left_trigger * .8)));
            } else {
                rightArm.setPower(0);
                leftArm.setPower(0);
            }

            leftLock.setPosition(.25);
            rightLock.setPosition(.4);


        } else if (!armPIDActive) {
            if (Math.abs(gamepad2.left_stick_y) > .02 || !cos) {
                rightArm.setPower(.5 * -gamepad2.left_stick_y * nitro2 * (1 - (gamepad2.left_trigger * .8)));
                leftArm.setPower(.5 * gamepad2.left_stick_y * nitro2 * (1 - (gamepad2.left_trigger * .8)));
            } else {
                theta = (((double) leftArm.getCurrentPosition())-1600) / (-6800.0);
                theta = theta * 3.14159;

                telemetry.addData("theta", theta);
                telemetry.addData("cos:", Math.cos(theta));
                leftArm.setPower(Math.cos(theta) * (.05+ .03*(armExtendLeft.getCurrentPosition()/1024)));
                rightArm.setPower(-Math.cos(theta) * (.05+ .03*(armExtendLeft.getCurrentPosition()/1024)));

            }
            leftLock.setPosition(0.37);
            rightLock.setPosition(.32);

        }
if(gamepad2.start){
            leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armExtendLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armExtendRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armExtendRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armExtendLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
}
telemetry.addData("cos",cos);

if(gamepad2.y && !press){
            cos=!cos;

}

if(gamepad2.y){
    press=true;
}
else{
    press=false;
}
        telemetry.addLine()
                .addData("IsLocked", isLocked)
                .addData("leftArmEncoder: ", leftArm.getCurrentPosition())
                .addData("rightArmEncoder: ", rightArm.getCurrentPosition())
                .addData("armExtendRightEncoder: ", armExtendRight.getCurrentPosition())
                .addData("armExtendLeftEncoder: ", armExtendLeft.getCurrentPosition());
        telemetry.update();

        if (gamepad2.dpad_up) {
            armSetPoint = -2600;
            if (leftArm.getCurrentPosition() < armSetPoint + 500 && leftArm.getCurrentPosition() > armSetPoint - 500) {
                armExtendSetPoint = 1024;
            } else {
                armExtendSetPoint = 33;
            }
            armPIDActive = true;
        } else if (gamepad2.dpad_down) {
            armSetPoint = -6286;
            if (leftArm.getCurrentPosition() < armSetPoint + 500 && leftArm.getCurrentPosition() > armSetPoint - 500) {
                armExtendSetPoint = 625;
            } else {
                armExtendSetPoint = 33;
            }

            armPIDActive = true;
        } else if (gamepad2.dpad_right) {
            armSetPoint = -232;
            armExtendSetPoint = 33;
            armPIDActive = true;
        } else {
            armPIDActive = false;
        }


        if (armPIDActive) {
            //-6652
            double armKp = 0.002;
            double armExtendKp = 0.002;

            int armError = (leftArm.getCurrentPosition()-1300) - armSetPoint;
            int armExtendError = armExtendLeft.getCurrentPosition() - armExtendSetPoint;

            double armPower = (double) armError * armKp;
            double armExtendPower = (double) armExtendError * armExtendKp;

            leftArm.setPower(armPower);
            rightArm.setPower(-armPower);
            armExtendLeft.setPower(armExtendPower);
            armExtendRight.setPower(-armExtendPower);

            telemetry.addData("Arm error", armError);
            telemetry.addData("Arm extend error", armExtendError);
        }

    }
}