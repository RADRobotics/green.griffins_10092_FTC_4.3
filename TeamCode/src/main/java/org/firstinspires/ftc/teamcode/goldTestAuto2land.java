/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Rect;
import org.opencv.core.Size;


@Autonomous(name="GoldTestAuto2Land", group="DogeCV")

public class goldTestAuto2land extends OpMode {
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



    double alignSize=50;
    double alignX    = (640 / 2) +0; // Center point in X Pixels
    double alignXMin = alignX - (alignSize / 2); // Min X Pos in pixels
    double alignXMax = alignX +(alignSize / 2); // Max X pos in pixels
    double scale = (640-alignSize)/2;

    //NORMALLY -4
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

        leftWheel = hardwareMap.dcMotor.get("leftWheel");
        rightWheel = hardwareMap.dcMotor.get("rightWheel");
        rightWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        leftArm = hardwareMap.dcMotor.get("leftArm");
        rightArm = hardwareMap.dcMotor.get("rightArm");

        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boolean unlock = true;




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
        }


        //moves forward
        if(stage==-3) {
            int targetPosition = (int) (.3 * TICKS_PER_WHEEL_ROTATION);
            rightWheel.setTargetPosition(targetPosition);
            leftWheel.setTargetPosition(targetPosition);
            leftWheel.setPower(1);
            rightWheel.setPower(1);
            if (runtime.seconds()>1) {
                stage=-2.5;
                leftArm.setDirection(DcMotorSimple.Direction.REVERSE);
                resetDriveEncoders();
                runtime.reset();
            }
        }
        //moves backwards
        if(stage==-2.5){
            rightArm.setPower(0);
            leftArm.setPower(0);
            int targetPosition = (int) (.5 * TICKS_PER_WHEEL_ROTATION);
            rightWheel.setTargetPosition(-targetPosition);
            leftWheel.setTargetPosition(-targetPosition);
            leftWheel.setPower(0.7);
            rightWheel.setPower(0.7);

            if ( runtime.seconds()>3) {
                stage=-2.6;
                leftArm.setDirection(DcMotorSimple.Direction.REVERSE);
                resetDriveEncoders();
                runtime.reset();
            }
        }

        //turns
        if(stage==-2.6){
            int targetPosition = (int) (.3 * TICKS_PER_WHEEL_ROTATION);
            rightWheel.setTargetPosition(targetPosition);
            leftWheel.setTargetPosition((int) ( -targetPosition * 1.5));
            leftWheel.setPower(1);
            rightWheel.setPower(1);

            if ((!leftWheel.isBusy() && !rightWheel.isBusy() )|| runtime.seconds()>3) {
                stage=-2;
                leftArm.setDirection(DcMotorSimple.Direction.REVERSE);
                resetDriveEncoders();
                runtime.reset();
            }
        }
        // leftArm.setDirection(DcMotorSimple.Direction.REVERSE);
///////////////////////////////////////////////////////////////ISSUE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //start vision procedure
        if(stage==-2) {
            rightArm.setTargetPosition((int) (.114 * TICKS_PER_WHEEL_ROTATION * 8));
            leftArm.setTargetPosition((int) (.114* TICKS_PER_WHEEL_ROTATION * 8));
            rightArm.setPower(.6);
            leftArm.setPower(.6);
            if(runtime.seconds()>4){
                stage=0;
            }
        }
;
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
            if(!detector.isFound() ){
                rightWheel.setPower(0);
                leftWheel.setPower(0);
                telemetry.addLine("not detected");
            }
            else if (xPos > alignXMax) {
                double power = ((xPos - alignXMax) / scale) * .3 + .4;
                rightWheel.setPower(-power);
                leftWheel.setPower(power);
                telemetry.addData("powL: ", power);
                telemetry.addLine("turning left");
                runtime.reset();
            } else if (xPos < alignXMin) {
                double power = ((alignXMin - xPos) / scale) * .3 + .4;
                rightWheel.setPower(power);
                leftWheel.setPower(-power);
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
            rightWheel.setTargetPosition(3*TICKS_PER_WHEEL_ROTATION);
            leftWheel.setTargetPosition(3*TICKS_PER_WHEEL_ROTATION);
            leftWheel.setPower(.5);
            rightWheel.setPower(.5);
            if(runtime.seconds()>5){
                rightWheel.setPower(0);
                leftWheel.setPower(0);
            }

        }


        telemetry.addData("stage: ", stage);
        telemetry.update();





        //end of l00p
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
       // telemetry.update();
    }
}