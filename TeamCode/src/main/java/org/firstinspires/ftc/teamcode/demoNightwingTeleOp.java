package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "demoNightwingTeleOp", group = "Competition")

public class demoNightwingTeleOp extends OpMode {
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
    private double nitro1;
    private double nitro2;
    private double nitro3;

    private Servo rightLock;
    private Servo leftLock;
    private Servo intake;
    private Servo intake2;

    private double setpoint = 0;

    private boolean isLocked = false;

    boolean safety=false;
    @Override
    public void init() {
        leftWheelFront = hardwareMap.dcMotor.get("leftWheel");
        leftWheelBack = hardwareMap.dcMotor.get("leftWheel2");
        rightWheelFront = hardwareMap.dcMotor.get("rightWheel");
        rightWheelBack = hardwareMap.dcMotor.get("rightWheel2");
        rightArm = hardwareMap.dcMotor.get("rightArm");
        leftArm = hardwareMap.dcMotor.get("leftArm");
        armExtendRight = hardwareMap.dcMotor.get("armExtend");
        armExtendLeft = hardwareMap.dcMotor.get("armExtend2");
        rightLock = hardwareMap.servo.get("rightLock");
        leftLock = hardwareMap.servo.get("leftLock");
        intake = hardwareMap.servo.get("intake");
        intake = hardwareMap.servo.get("intake2");

        rightLock.setPosition(.6);
        leftLock.setPosition(.4);

//        leftWheelFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftWheelBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightWheelFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightWheelBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        leftWheelFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftWheelBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightWheelFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightWheelBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

    @Override
    public void loop() {
        if(!safety) {
            nitro1 = .5 + (gamepad1.right_trigger * .5);
            nitro2 = 1 + (gamepad2.right_trigger);
            nitro3 = 1 - (gamepad2.left_trigger * .8);


            //drive motors
            rightWheelFront.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x) * nitro1);
            rightWheelBack.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x) * nitro1);
            leftWheelBack.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x) * nitro1);
            leftWheelFront.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x) * nitro1);

            //arm extension motors
            if (gamepad1.dpad_down) {
                armExtendLeft.setPower(.5 + (gamepad1.right_trigger * .5));
                armExtendRight.setPower(-(.5 + (gamepad1.right_trigger * .5)));
            }
            else if (gamepad1.dpad_up) {
                armExtendLeft.setPower(-(.5 + (gamepad1.right_trigger * .5)));
                armExtendRight.setPower(.5 + (gamepad1.right_trigger * .5));

            }
            else{
                armExtendLeft.setPower(0);
                armExtendRight.setPower(0);
            }
            // armExtendRight.setPower(.5*(-gamepad2.right_stick_y) *nitro2*(1-(gamepad2.left_trigger*.8)));
            //armExtendLeft.setPower(.5*(gamepad2.right_stick_y )*nitro2*(1-(gamepad2.left_trigger*.8)));
            //arm motors/lock servos
//        setpoint = /*rightArm.getCurrentPosition()+*/ gamepad2.left_stick_y*125*(nitro2);

//        if (gamepad1.a) {
//            isLocked = true;
//        }
//        if(gamepad1.b){
//            isLocked = false;
//        }
            if (isLocked) {
                if (gamepad2.left_stick_y < 0) {
                    rightArm.setPower(.5 * -gamepad2.right_stick_y * nitro2 * (1 - (gamepad2.left_trigger * .8)));
                    leftArm.setPower(.5 * gamepad2.right_stick_y * nitro2 * (1 - (gamepad2.left_trigger * .8)));
                } else {
                    rightArm.setPower(0);
                    leftArm.setPower(0);
                }
                //leftLock.setPosition(.25);
                //rightLock.setPosition(.45);
            } else {
                if (gamepad1.right_bumper) {
                    rightArm.setPower(.25 * -gamepad1.right_stick_y * nitro2 * (1 - (gamepad1.left_trigger * .8)));
                    leftArm.setPower(.25 * gamepad1.right_stick_y * nitro2 * (1 - (gamepad1.left_trigger * .8)));

                } else {
                    rightArm.setPower(.5 * -gamepad1.right_stick_y * nitro2 * (1 - (gamepad1.left_trigger * .8)));
                    leftArm.setPower(.5 * gamepad1.right_stick_y * nitro2 * (1 - (gamepad1.left_trigger * .8)));
                }

                //leftLock.setPosition(0.5);
                //rightLock.setPosition(.3);
            }


        }
        else{
            nitro1 = .5 + (gamepad2.right_trigger * .5);
            nitro2 = 1 + (gamepad2.right_trigger);
            nitro3 = 1 - (gamepad2.left_trigger * .8);


            //drive motors
            rightWheelFront.setPower((-gamepad2.left_stick_y + gamepad2.left_stick_x) * nitro1);
            rightWheelBack.setPower((-gamepad2.left_stick_y + gamepad2.left_stick_x) * nitro1);
            leftWheelBack.setPower((gamepad2.left_stick_y + gamepad2.left_stick_x) * nitro1);
            leftWheelFront.setPower((gamepad2.left_stick_y + gamepad2.left_stick_x) * nitro1);

            //arm extension motors
            if (gamepad2.dpad_down) {
                armExtendLeft.setPower(.5 + (gamepad2.right_trigger * .5));
                armExtendRight.setPower(-(.5 + (gamepad2.right_trigger * .5)));
            }
           else if (gamepad2.dpad_up) {
                armExtendLeft.setPower(-(.5 + (gamepad2.right_trigger * .5)));
                armExtendRight.setPower(.5 + (gamepad2.right_trigger * .5));

            }
            else{
                armExtendLeft.setPower(0);
                armExtendRight.setPower(0);
            }
            // armExtendRight.setPower(.5*(-gamepad2.right_stick_y) *nitro2*(1-(gamepad2.left_trigger*.8)));
            //armExtendLeft.setPower(.5*(gamepad2.right_stick_y )*nitro2*(1-(gamepad2.left_trigger*.8)));
            //arm motors/lock servos
//        setpoint = /*rightArm.getCurrentPosition()+*/ gamepad2.left_stick_y*125*(nitro2);

//        if (gamepad2.a) {
//            isLocked = true;
//        }
//        if(gamepad2.b){
//            isLocked = false;
//        }
            if (isLocked) {
                if (gamepad2.left_stick_y < 0) {
                    rightArm.setPower(.5 * -gamepad2.right_stick_y * nitro2 * (1 - (gamepad2.left_trigger * .8)));
                    leftArm.setPower(.5 * gamepad2.right_stick_y * nitro2 * (1 - (gamepad2.left_trigger * .8)));
                } else {
                    rightArm.setPower(0);
                    leftArm.setPower(0);
                }
                //leftLock.setPosition(.25);
                //rightLock.setPosition(.45);
            } else {
                if (gamepad2.right_bumper) {
                    rightArm.setPower(.25 * -gamepad2.right_stick_y * nitro2 * (1 - (gamepad2.left_trigger * .8)));
                    leftArm.setPower(.25 * gamepad2.right_stick_y * nitro2 * (1 - (gamepad2.left_trigger * .8)));

                } else {
                    rightArm.setPower(.5 *   -gamepad2.right_stick_y * nitro2 * (1 - (gamepad2.left_trigger * .8)));
                    leftArm.setPower(.5 * gamepad2.right_stick_y * nitro2 * (1 - (gamepad2.left_trigger * .8)));
                }

                //leftLock.setPosition(0.5);
                //rightLock.setPosition(.3);
            }








        }

        if (gamepad1.left_bumper){
            leftLock.setPosition(.75);
            telemetry.addData("test2", gamepad1.left_bumper);
        }else{
            leftLock.setPosition(0);
        }

        if (gamepad1.right_bumper){
            rightLock.setPosition(0.75);
            telemetry.addData("test", gamepad1.right_bumper);
        }else{
            rightLock.setPosition(0);
        }


        if(gamepad2.a){
            safety=true;
        }
        if(gamepad2.b){
            safety=false;
        }
                telemetry.addData("IsLocked", isLocked);

                telemetry.update();


            }}