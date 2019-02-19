package org.firstinspires.ftc.teamcode.comp_code;

import android.media.AudioManager;
import android.media.SoundPool;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.utils.hmap;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Competition")

public class TeleOp extends OpMode {
    hmap hwmap = new hmap();

    private double nitro1;
    private double nitro2;

    boolean yeaboi = false;
    boolean cos = false;
    boolean press = false;

    private boolean isLocked = false;

    private int armSetPoint = 0;
    private int armExtendSetPoint = 0;
    private double power = 0;
    private boolean armPIDActive = false;
    double theta;


    public SoundPool mySound;
    public int beepID;
    int streamID;
    int streamIDy;
    boolean play = false;

int yeahboi;
    @Override
    public void init() {
        hwmap.init(hardwareMap);

        mySound= new SoundPool(1,AudioManager.STREAM_MUSIC,0);
        beepID = mySound.load(hardwareMap.appContext, R.raw.mario,1);
        yeahboi = mySound.load(hardwareMap.appContext, R.raw.yeah_boi,1);

        telemetry.update();


    }

    @Override
    public void loop() {

        telemetry.addData("range", String.format("%.01f mm", hwmap.sensorRange.getDistance(DistanceUnit.MM)));

        //mario sounds nitro!!!
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
//        nitro1 = .6 + (gamepad1.right_trigger * .4) - (gamepad1.left_trigger*.5);
//        nitro2 = 1 + ((gamepad2.right_trigger));

        nitro1=0.6;
        if(gamepad1.right_bumper){
           nitro1 = 1;
        }

        if (gamepad1.left_bumper){
            nitro2 = .2;
        }


        //drive motors
        if(!(gamepad1.right_trigger>.05 || gamepad1.left_trigger>.05)) {
            hwmap.rs((-gamepad1.left_stick_y - gamepad1.right_stick_x) * nitro1);
            hwmap.ls((-gamepad1.left_stick_y + gamepad1.right_stick_x) * nitro1);
        }
        else{
            if(!gamepad1.b) {
                hwmap.rs(gamepad1.right_trigger * nitro1);
                hwmap.ls(-gamepad1.left_trigger * nitro1);
            }
            else{
                hwmap.rs(-gamepad1.right_trigger * nitro1);
                hwmap.ls(gamepad1.left_trigger * nitro1);
            }

        }

        //arm extension motors
        if (!armPIDActive) {
            hwmap.armExtendRight.setPower(.5 * (gamepad2.right_stick_y) * nitro2 * (1 - (gamepad2.left_trigger * .8)));
            hwmap.armExtendLeft.setPower(.5 * (gamepad2.right_stick_y) * nitro2 * (1 - (gamepad2.left_trigger * .8)));
        }

        //intake
        if (gamepad2.left_bumper) {
            if (gamepad2.b) {
                hwmap.intake.setPosition(0.25);
            } else {
                hwmap.intake.setPosition(.75);
            }
            telemetry.addData("test2", gamepad1.left_bumper);
        } else {
            hwmap.intake.setPosition(0);
        }

        if (gamepad2.right_bumper) {
            if (gamepad2.b) {
                hwmap.intake2.setPosition(0.25);
            } else {
                hwmap.intake2.setPosition(0.75);
            }
            telemetry.addData("test", gamepad1.right_bumper);
        } else {
            hwmap.intake2.setPosition(0);
        }

        //ratchet stuff/arm
        if (gamepad2.x) {
            isLocked = true;
            armPIDActive=true;
            armSetPoint=-100;
            armExtendSetPoint=700;
        }
        if (gamepad2.a) {
            isLocked = false;
            armPIDActive=false;
        }
//        if(cos){
//            hwmap.leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//            hwmap.rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        }
//        else{
//                hwmap.leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                hwmap.rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        }
        if (isLocked) {
            if (gamepad2.left_stick_y < 0) {
                hwmap.arm(-(.5 * gamepad2.left_stick_y * nitro2 * (1 - (gamepad2.left_trigger * .8))));
               // hwmap.leftArm.setPower(.5 * gamepad2.left_stick_y * nitro2 * (1 - (gamepad2.left_trigger * .8)));
            } else {
                hwmap.arm(0);
            }
        }
        else if (!armPIDActive) {

                hwmap.arm(-(.5 * gamepad2.left_stick_y * nitro2 * (1 - (gamepad2.left_trigger * .8))));
                //hwmap.leftArm.setPower(.5 * gamepad2.left_stick_y * nitro2 * (1 - (gamepad2.left_trigger * .8)));
//                theta = (((double) hwmap.leftArm.getCurrentPosition())-1600) / (-6800.0);
//                theta = theta * 3.14159;
//
//                telemetry.addData("theta", theta);
//                telemetry.addData("cos:", Math.cos(theta));
               // hwmap.leftArm.setPower(Math.cos(theta) * (.05+ .03*(hwmap.armExtendLeft.getCurrentPosition()/1024)));
                //hwmap.rightArm.setPower(Math.cos(theta) * (.05+ .03*(hwmap.armExtendLeft.getCurrentPosition()/1024)));



        }

        //lock/unlock
            hwmap.lock(isLocked);

        //reset encoders
if(gamepad2.start && gamepad2.x){
            hwmap.reset();
}


if(gamepad2.y && !press){
            cos=!cos;
}

if(gamepad2.y){
    press=true;
}
else{
    press=false;
}

        telemetry.addData("cos",cos);
        telemetry.addLine()
                .addData("dat", hwmap.print());
        telemetry.update();

        //setpoints
        if (gamepad2.dpad_up) {
            hwmap.leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hwmap.rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armSetPoint = 1700;
            if (hwmap.leftArm.getCurrentPosition() < armSetPoint + 800 && hwmap.leftArm.getCurrentPosition() > armSetPoint - 800) {
                armExtendSetPoint = 950;
            } else {
                armExtendSetPoint = 250;
            }
            armPIDActive = true;
        } else if (gamepad2.dpad_down) {
            hwmap.leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hwmap.rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armSetPoint = 5066;
            if (hwmap.leftArm.getCurrentPosition() < armSetPoint + 800 && hwmap.leftArm.getCurrentPosition() > armSetPoint - 800) {
                armExtendSetPoint = 450;
            } else {
                armExtendSetPoint = 250;
            }

            armPIDActive = true;
        } else if (gamepad2.dpad_right) {
            hwmap.leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            hwmap.rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            armSetPoint = 3000;
            armExtendSetPoint = 33;
            armPIDActive = true;
        } else if((gamepad2.dpad_left)) {
            hwmap.leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hwmap.rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armSetPoint = -100;

                armExtendSetPoint = 250;


            armPIDActive = true;
        } else {
            armPIDActive = false;
        }
        if((gamepad1.dpad_left) && !yeaboi){
            streamIDy= mySound.play(yeahboi,1,1,1,-1,1);
        }
        if(gamepad1.dpad_left){
            yeaboi=true;
        }
        else{
            yeaboi=false;
            mySound.stop(streamIDy);
        }


        if (armPIDActive) {
            //-6652
            double armKp = 0.002*nitro2;
            double armExtendKp = 0.002 * nitro2;

            int armError = (hwmap.leftArm.getCurrentPosition()) - armSetPoint;
            int armExtendError = hwmap.armExtendLeft.getCurrentPosition() - armExtendSetPoint;

            double armPower = (double) armError * armKp;
            double armExtendPower = (double) armExtendError * armExtendKp;
if(armExtendPower>.6 &&!gamepad2.dpad_right){
    armExtendPower=.6;
}
if(armExtendPower<-.6&&!gamepad2.dpad_right){
    armExtendPower=-.6;
     }
            hwmap.leftArm.setPower(armPower);
            hwmap.rightArm.setPower(armPower);
            hwmap.armExtendLeft.setPower(armExtendPower);
            hwmap.armExtendRight.setPower(armExtendPower);

            telemetry.addData("Arm error", armError);
            telemetry.addData("Arm extend error", armExtendError);
        }

//        if(gamepad1.a){
//            power = (0.01*hwmap.sensorRange.getDistance(DistanceUnit.MM)- 500);
//            telemetry.addData("Power: ",power);
//    }

        telemetry.update();

    }
    public void stop(){
        mySound.stop(streamIDy);
        mySound.stop(streamID);
    }
}


//add lidar thing