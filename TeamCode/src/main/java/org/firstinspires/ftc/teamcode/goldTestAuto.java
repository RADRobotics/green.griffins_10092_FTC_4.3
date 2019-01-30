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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Size;


@Autonomous(name="GoldTestAuto", group="DogeCV")

public class goldTestAuto extends OpMode {
    // Detector object
    private GoldDetector detector;

    private DcMotor leftWheel;
    private DcMotor rightWheel;


    double alignSize=120;
    double alignX    = (640 / 2) +0; // Center point in X Pixels
    double alignXMin = alignX - (alignSize / 2); // Min X Pos in pixels
    double alignXMax = alignX +(alignSize / 2); // Max X pos in pixels

    boolean aligned = false;
    @Override
    public void init() {
        leftWheel = hardwareMap.dcMotor.get("leftWheel");
        rightWheel = hardwareMap.dcMotor.get("rightWheel");
        leftWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

    }

    /*
     * Code to run REPEATEDLY when the driver hits PLAY
     */
    @Override
    public void loop() {
//        telemetry.addData("IsAligned" , detector.getAligned()); // Is the bot aligned with the gold mineral?
//        telemetry.addData("X Pos" , detector.getXPosition()); // Gold X position.

        //Point screenpos = detector.getScreenPosition();
        Rect bestRect = detector.getFoundRect();

        double xPos = bestRect.x + (bestRect.width / 2);

        if(xPos < alignXMax && xPos > alignXMin){
            aligned = true;
        }else{
            aligned = false;
        }
        telemetry.addData("aligned ",aligned);

        telemetry.addData("xpos ",xPos);
        telemetry.addData("amax ",alignXMax);
        telemetry.addData("amin ",alignXMin);

        if(xPos>alignXMax){
            rightWheel.setPower(.6);
            leftWheel.setPower(-.6);
            telemetry.addLine("turning left");
        }
        else if(xPos<alignXMin){
            rightWheel.setPower(-.6);
            leftWheel.setPower(.6);
            telemetry.addLine("turning right");
        }
        else{
            rightWheel.setPower(0);
            leftWheel.setPower(0);
            telemetry.addLine("found");
        }
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        detector.disable();
    }

}
