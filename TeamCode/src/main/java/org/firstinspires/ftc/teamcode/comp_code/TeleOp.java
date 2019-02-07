package org.firstinspires.ftc.teamcode.comp_code;

import android.media.AudioManager;
import android.media.SoundPool;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.utils.hmap;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Competition")

public class TeleOp extends OpMode {
    hmap hwmap = new hmap();

    private double nitro1;
    private double nitro2;

    boolean cos = false;
    boolean press = false;

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
        hwmap.init(hardwareMap);

        mySound= new SoundPool(1,AudioManager.STREAM_MUSIC,0);
        beepID = mySound.load(hardwareMap.appContext, R.raw.mario,1);

    }

    @Override
    public void loop() {

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
        nitro1 = .6 + (gamepad1.right_trigger * .4) - (gamepad1.left_trigger*.5);
        nitro2 = 1 + ((gamepad2.right_trigger));


        //drive motors
           hwmap.rs((-gamepad1.left_stick_y - gamepad1.right_stick_x) * nitro1);
            hwmap.ls((-gamepad1.left_stick_y + gamepad1.right_stick_x) * nitro1);

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
        }
        if (gamepad2.a) {
            isLocked = false;
        }
        if (isLocked) {
            if (gamepad2.left_stick_y < 0) {
                hwmap.arm(-(.5 * gamepad2.left_stick_y * nitro2 * (1 - (gamepad2.left_trigger * .8))));
               // hwmap.leftArm.setPower(.5 * gamepad2.left_stick_y * nitro2 * (1 - (gamepad2.left_trigger * .8)));
            } else {
                hwmap.arm(0);
            }
        }
        else if (!armPIDActive) {
            if (Math.abs(gamepad2.left_stick_y) > .02 || !cos) {
                hwmap.arm(-(.5 * gamepad2.left_stick_y * nitro2 * (1 - (gamepad2.left_trigger * .8))));
                //hwmap.leftArm.setPower(.5 * gamepad2.left_stick_y * nitro2 * (1 - (gamepad2.left_trigger * .8)));
            } else {
                theta = (((double) hwmap.leftArm.getCurrentPosition())-1600) / (-6800.0);
                theta = theta * 3.14159;

                telemetry.addData("theta", theta);
                telemetry.addData("cos:", Math.cos(theta));
               // hwmap.leftArm.setPower(Math.cos(theta) * (.05+ .03*(hwmap.armExtendLeft.getCurrentPosition()/1024)));
                //hwmap.rightArm.setPower(Math.cos(theta) * (.05+ .03*(hwmap.armExtendLeft.getCurrentPosition()/1024)));

            }

        }

        //lock/unlock
            hwmap.lock(isLocked);

        //reset encoders
if(gamepad2.start){
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
            armSetPoint = -2600;
            if (hwmap.leftArm.getCurrentPosition() < armSetPoint + 500 && hwmap.leftArm.getCurrentPosition() > armSetPoint - 500) {
                armExtendSetPoint = 1024;
            } else {
                armExtendSetPoint = 33;
            }
            armPIDActive = true;
        } else if (gamepad2.dpad_down) {
            armSetPoint = -6286;
            if (hwmap.leftArm.getCurrentPosition() < armSetPoint + 500 && hwmap.leftArm.getCurrentPosition() > armSetPoint - 500) {
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

            int armError = (hwmap.leftArm.getCurrentPosition()-1300) - armSetPoint;
            int armExtendError = hwmap.armExtendLeft.getCurrentPosition() - armExtendSetPoint;

            double armPower = (double) armError * armKp;
            double armExtendPower = (double) armExtendError * armExtendKp;

            hwmap.leftArm.setPower(armPower);
            hwmap.rightArm.setPower(armPower);
            hwmap.armExtendLeft.setPower(armExtendPower);
            hwmap.armExtendRight.setPower(armExtendPower);

            telemetry.addData("Arm error", armError);
            telemetry.addData("Arm extend error", armExtendError);
        }

    }
}