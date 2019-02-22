package org.firstinspires.ftc.teamcode.comp_code;

import android.media.AudioManager;
import android.media.SoundPool;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.utils.hmap;

import java.io.File;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "record_auto_gyro", group = "Competition")

public class Record_auto_gyro extends LinearOpMode {
    hmap hwmap = new hmap();

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime totalTime = new ElapsedTime();
    int intake=0;
    int intake2=0;
    String thing = "";

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

    int record = 0;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double                  globalAngle;


    @Override
    public void runOpMode() throws InterruptedException {
        hwmap.init(hardwareMap);
        //hwmap.initGyro();
        hwmap.reset();
        mySound = new SoundPool(1, AudioManager.STREAM_MUSIC, 0);
        beepID = mySound.load(hardwareMap.appContext, R.raw.mario, 1);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        waitForStart();
        resetAngle();

        while (opModeIsActive()) {


            if ((gamepad1.right_stick_button || gamepad2.right_stick_button) && record == 0) {
                record = 1;
            }
            if (record == 1 && !gamepad1.right_stick_button && !gamepad2.right_stick_button) {
                record = 2;
            }
            if (record == 0) {
                telemetry.addData("Press right_stick_button to begin recording", "");
            }
            if (record == 2) {
                if ((gamepad1.right_stick_button || gamepad2.right_stick_button)) {
                    record = 3;
                }
                telemetry.addData("recording, runtime: ", totalTime.seconds());
                telemetry.addData("press right_stick_button to stop recording", "");
                if (runtime.seconds() > .025) {
                    //list.add(runtime2.seconds() + " " + rw.getCurrentPosition() + " " + lw.getCurrentPosition());
                    //thing += "s:"+runtime2.seconds() + " r:" + rw.getCurrentPosition() + " l:" + lw.getCurrentPosition() + "; ";
                    if (gamepad2.right_bumper) {
                        intake = 1;
                        if(gamepad2.b){
                            intake=-1;
                        }
                    } else {
                        intake = 0;
                    }
                    if (gamepad2.left_bumper) {
                        intake2 = 1;
                        if(gamepad2.b){
                            intake2=-1;
                        }
                    } else {
                        intake2 = 0;
                    }
                    //6
                    thing += (hwmap.rw1.getCurrentPosition()) + "," + (hwmap.lw1.getCurrentPosition()) + "," + (int)(getAngle()*1000) + "," + hwmap.leftArm.getCurrentPosition() + "," + hwmap.armExtendLeft.getCurrentPosition() + "," + intake + "," + intake2 +"@";//hwmap.gyro.getHeading()*1000
                    telemetry.addData("sent", "");
                    runtime.reset();
                }
            }
            if (record == 3) {
                if (gamepad1.x || gamepad2.x) {
                    String filename = "left.csv";
                    File file = AppUtil.getInstance().getSettingsFile(filename);
                    ReadWriteFile.writeFile(file, thing);
                    telemetry.log().add("saved to '%s'", filename);
                    record = 0;
                }
                if (gamepad1.a || gamepad2.a) {
                    String filename = "center.csv";
                    File file = AppUtil.getInstance().getSettingsFile(filename);
                    ReadWriteFile.writeFile(file, thing);
                    telemetry.log().add("saved to '%s'", filename);
                    record = 0;
                }
                if (gamepad1.b || gamepad2.b) {
                    String filename = "right.csv";
                    File file = AppUtil.getInstance().getSettingsFile(filename);
                    ReadWriteFile.writeFile(file, thing);
                    telemetry.log().add("saved to '%s'", filename);
                    record = 0;
                }
                if (gamepad1.y || gamepad2.y) {
                    telemetry.log().add("Deleted");
                    thing = "";
                    record = 0;
                }
                telemetry.addData("press X to save to LEFT, A to CENTER, B to RIGHT\npress Y to DELETE", "");
            }
            telemetry.addData("recordvar", record);
            //mario sounds nitro!!!





            if ((gamepad1.right_trigger > .1 || gamepad2.right_trigger > .1) && play == false) {
                streamID = mySound.play(beepID, 1, 1, 1, -1, 1);
                play = true;
            } else if (gamepad1.right_trigger > .1 || gamepad2.right_trigger > .1) {

            } else {
                play = false;
                mySound.stop(streamID);
            }
            nitro1 = .6 + (gamepad1.right_trigger * .4) - (gamepad1.left_trigger * .5);
            nitro2 = 1 + (gamepad2.right_trigger);


            //drive motors
            hwmap.rw1.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x) * nitro1);
            hwmap.rw2.setPower((gamepad1.left_stick_y + gamepad1.right_stick_x) * nitro1);
            hwmap.lw1.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x) * nitro1);
            hwmap.lw2.setPower((gamepad1.left_stick_y - gamepad1.right_stick_x) * nitro1);

            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", getAngle());

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
                    hwmap.rightArm.setPower(.5 * gamepad2.left_stick_y * nitro2 * (1 - (gamepad2.left_trigger * .8)));
                    hwmap.leftArm.setPower(.5 * gamepad2.left_stick_y * nitro2 * (1 - (gamepad2.left_trigger * .8)));
                } else {
                    hwmap.rightArm.setPower(0);
                    hwmap.leftArm.setPower(0);
                }
            } else if (!armPIDActive) {
                if (Math.abs(gamepad2.left_stick_y) > .02 || !cos) {
                    hwmap.rightArm.setPower(.5 * gamepad2.left_stick_y * nitro2 * (1 - (gamepad2.left_trigger * .8)));
                    hwmap.leftArm.setPower(.5 * gamepad2.left_stick_y * nitro2 * (1 - (gamepad2.left_trigger * .8)));
                } else {
                    theta = (((double) hwmap.leftArm.getCurrentPosition()) - 1600) / (-6800.0);
                    theta = theta * 3.14159;

                    telemetry.addData("theta", theta);
                    telemetry.addData("cos:", Math.cos(theta));
                    hwmap.leftArm.setPower(Math.cos(theta) * (.05 + .03 * (hwmap.armExtendLeft.getCurrentPosition() / 1024)));
                    hwmap.rightArm.setPower(Math.cos(theta) * (.05 + .03 * (hwmap.armExtendLeft.getCurrentPosition() / 1024)));

                }

            }

            //lock/unlock
            hwmap.lock(isLocked);

            //reset encoders
            if (gamepad2.start) {
                hwmap.reset();
            }


            if (gamepad2.y && !press) {
                cos = !cos;
            }

            if (gamepad2.y) {
                press = true;
            } else {
                press = false;
            }

            telemetry.addData("cos", cos);
            telemetry.addLine()
                    .addData("dat", hwmap.print());
            //telemetry.addData("gyro",hwmap.gyro.getHeading());
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

                int armError = (hwmap.leftArm.getCurrentPosition() - 1300) - armSetPoint;
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
        //exit while
    }
    //methods here

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }



}