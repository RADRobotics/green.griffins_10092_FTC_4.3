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

package org.firstinspires.ftc.teamcode.comp_code;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.disnodeteam.dogecv.filters.HSVColorFilter;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.hmap;


@Autonomous(name="Auto test", group="DogeCV")

public class Auto extends OpMode {
    // Detector object
    private SamplingOrderDetector detector;
    SamplingOrderDetector.GoldLocation order;
    private ElapsedTime runtime = new ElapsedTime();
    hmap hwmap = new hmap();
    int stage = 0;

    @Override
    public void init() {
        hwmap.init(hardwareMap);
        telemetry.addData("Status", "DogeCV 2018.0 - Sampling Order Example");

        // Setup detector
        detector = new SamplingOrderDetector(); // Create the detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize detector with app context and camera
        detector.useDefaults(); // Set detector to use default settings

        detector.downscale = 0.4; // How much to downscale the input frames

        // Optional tuning
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.001;

        detector.ratioScorer.weight = 15;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable(); // Start detector
    }

    /*
     * Code to run REPEATEDLY when the driver hits INIT
     */
    @Override
    public void init_loop() {
        telemetry.update();
        telemetry.addData("Current Order" , detector.getCurrentOrder().toString()); // The current result for the frame
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        order = detector.getLastOrder();
        hwmap.reset();
        runtime.reset();
    }


    /*
     * Code to run REPEATEDLY when the driver hits PLAY
     */
    @Override
    public void loop() {
        telemetry.update();
        telemetry.addData("enc:  ", hwmap.print());
        telemetry.addData("stage: ",stage);
        telemetry.addData("Current Order" , detector.getCurrentOrder().toString()); // The current result for the frame
        telemetry.addData("Last Order" , order.toString()); // The last known result

        if(stage==0){
            hwmap.arm(-.5);
            hwmap.lock(false);
            if(hwmap.leftArm.getCurrentPosition()>200 || runtime.seconds()>2){
                stage++;
                hwmap.zero();
                runtime.reset();

            }
        }
        if(stage==1){
            hwmap.arm(.05);
            if(runtime.seconds()>4){
                stage=0;
                runtime.reset();
                hwmap.zero();
            }
            if(hwmap.leftArm.getCurrentPosition()<-3500){
                stage++;
                hwmap.zero();
                runtime.reset();
            }
        }
        if(stage==2){
            hwmap.arm(.25);
            if(hwmap.leftArm.getCurrentPosition()<-4000|| runtime.seconds()>2){
                stage++;
                hwmap.zero();
                runtime.reset();
            }
        }

        if(stage==3){
            hwmap.arm(-.05);
            if(runtime.seconds()>3){
                stage=-1;
                hwmap.zero();
                runtime.reset();
            }
            if(hwmap.leftArm.getCurrentPosition()>-1000){
                stage++;
                hwmap.zero();
                runtime.reset();
            }
        }
        if(stage==-1){
            hwmap.arm(-1);
            if(hwmap.leftArm.getCurrentPosition()>-3500){
                stage=2;
                hwmap.zero();
                runtime.reset();
            }
        }
        if(stage==4){

        }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        detector.disable();
    }

}
