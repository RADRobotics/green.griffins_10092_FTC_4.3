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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.utils.hmap;
import org.opencv.core.Rect;
import org.opencv.core.Size;

import java.io.File;

import static java.lang.Integer.parseInt;


@Autonomous(name="Auto test Gold", group="DogeCV")

public class autoGoldDetect extends OpMode {
    // Detector object
    private GoldDetector detector;
    SamplingOrderDetector.GoldLocation order= SamplingOrderDetector.GoldLocation.LEFT;
    private ElapsedTime runtime = new ElapsedTime();
    hmap hwmap = new hmap();
    int stage = 0;
int[][] data;
int[][] dataLeft;
int[][] dataRight;
int[][] dataCenter;


    int pos = 0;

    int setL = 0;
    int setR = 0;

    int speedL = 0;
    int speedR = 0;

    double gyro = 0.0;
    int arm = 0;
    int extend = 0;
    int intake = 0;
    int intake2 = 0;

    int errorR;
    int errorL;

    int sumErrorR;
    int sumErrorL;

    int previousErrorR;
    int previousErrorL;

    //double Kp = .002;
    //double Ki = 0.0001;
    //double Kd = 0.001;

    double Kp = 0.001;
    double Ki = 0.00006;
    double Kd = 0;
    double Kg = 0.05;

    double Kf = 0.009;
double pow = 0;
    @Override
    public void init() {

        telemetry.addData("reading left...","");
        String readfile = "left.csv";
        File fileR = AppUtil.getInstance().getSettingsFile(readfile);
        String sdata = ReadWriteFile.readFile(fileR);
        String[] split1 = sdata.split("@");
        dataLeft = new int[split1.length][7];
        for (int i=0;i<split1.length-1;i++){
            String[] split2 = split1[i].split(",");
            for(int p=0;p<split2.length;p++){
                dataLeft[i][p]=parseInt(split2[p]);
            }

        }

        telemetry.addData("reading center...","");
        File fileR2 = AppUtil.getInstance().getSettingsFile("center.csv");
        String sdata2 = ReadWriteFile.readFile(fileR2);
        String[] split1c = sdata2.split("@");
         dataCenter = new int[split1c.length][7];
        for (int i=0;i<split1c.length-1;i++){
            String[] split2 = split1c[i].split(",");
            for(int p=0;p<split2.length;p++){
                dataCenter[i][p]=parseInt(split2[p]);
            }

        }

        telemetry.addData("reading right...","");
        File fileRr = AppUtil.getInstance().getSettingsFile("right.csv");
        String sdatar = ReadWriteFile.readFile(fileRr);
        String[] split1r = sdatar.split("@");
        dataRight = new int[split1r.length][7];
        for (int i=0;i<split1r.length-1;i++){
            String[] split2 = split1r[i].split(",");
            for(int p=0;p<split2.length;p++){
                dataRight[i][p]=parseInt(split2[p]);
            }

        }

        telemetry.addData("done reading","");



        hwmap.init(hardwareMap);
        telemetry.addData("Status", "DogeCV 2018.0 - Sampling Order Example");

        // Setup detector
        detector = new GoldDetector(); // Create the detector
        detector.setAdjustedSize(new Size(480, 270));
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(),1,false); // Initialize detector with app context and camera
        detector.useDefaults(); // Set detector to use default settings

        detector.downscale = 0.4; // How much to downscale the input frames

        // Optional tuning
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
        Rect bestRect = detector.getFoundRect();
        double xPos = bestRect.x + (bestRect.width / 2);
        telemetry.addData("pos:",order.toString());
        if (xPos > 0) {
            if (xPos < 213) {
                order= SamplingOrderDetector.GoldLocation.LEFT;
            } else if (xPos < 427) {
                order= SamplingOrderDetector.GoldLocation.CENTER;
            } else if (xPos < 640) {
                order= SamplingOrderDetector.GoldLocation.RIGHT;
            }
        }
        telemetry.update();
        //telemetry.addData("Current Order" , detector.getCurrentOrder().toString()); // The current result for the frame
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        //order = detector.getLastOrder();
        //if(order== )
        if(order==SamplingOrderDetector.GoldLocation.LEFT){
            data=dataLeft;
        }
        if(order==SamplingOrderDetector.GoldLocation.CENTER){
            data=dataCenter;
        }
        if(order==SamplingOrderDetector.GoldLocation.RIGHT){
            data=dataRight;
        }
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
        //telemetry.addData("Current Order" , detector.getCurrentOrder().toString()); // The current result for the frame
        telemetry.addData("Last Order" , order.toString()); // The last known result
telemetry.addData("runtime",runtime);
        telemetry.addData("pow",pow);
        if(stage==0){
            pow=-.4;
            hwmap.lock(false);
            if(hwmap.leftArm.getCurrentPosition()>300|| runtime.seconds()>2){
                stage=1;
                //hwmap.leftArm.setPower(0);
                //hwmap.rightArm.setPower(0);
                runtime.reset();

            }
        }
        if(stage==1){
            //hwmap.arm(.05);
           pow=0.05;
//            if(runtime.seconds()>2 && hwmap.leftArm.getCurrentPosition()>-100){
//                stage=0;
//                runtime.reset();
//                hwmap.zero();
//            }
            if(hwmap.leftArm.getCurrentPosition()<-3000){
                stage++;
               // hwmap.leftArm.setPower(0);
                //hwmap.rightArm.setPower(0);
                runtime.reset();
            }
        }
        if(stage==2){
            pow=.2;
            if(hwmap.leftArm.getCurrentPosition()<-3500|| runtime.seconds()>5){
                stage++;
                //hwmap.leftArm.setPower(0);
                //hwmap.rightArm.setPower(0);
                runtime.reset();
            }
        }

        if(stage==3){
            pow=-.1;
//            if(runtime.seconds()>8 && hwmap.leftArm.getCurrentPosition()<-3000){
//                stage=-1;
//                hwmap.zero();
//                runtime.reset();
//            }
            if(hwmap.leftArm.getCurrentPosition()>-300){
                stage++;
                pow = 0;
                runtime.reset();
            }
        }
        hwmap.leftArm.setPower(pow);
        hwmap.rightArm.setPower(pow);
//        if(stage==-1){
//            hwmap.arm(-1);
//            if(hwmap.leftArm.getCurrentPosition()>-2500){
//                stage=1;
//                hwmap.zero();
//                runtime.reset();
//            }
//        }
        if(stage==4){
        if (runtime.seconds() > .025) {
            setL = data[pos][1];
            setR = data[pos][0];
            speedL =  data[pos+1][1] - data[pos][1];
            speedR =  data[pos+1][0] - data[pos][0];
            gyro = ((double)data[pos][2])/1000;
            arm = data[pos][3];
            extend = data[pos][4];
            intake= data[pos][5];
            intake2= data[pos][6];
            if(pos<data.length-2) {
                pos++;
            }
            runtime.reset();
        }
        errorR= setR-hwmap.rw1.getCurrentPosition();
        errorL = setL-hwmap.lw1.getCurrentPosition();
        double pr;
        double pl;


        double dErrorR = errorR - previousErrorR;
        double dErrorL = errorL - previousErrorL;

        double gyroError =0;//gyro-hwmap.gyro.getHeading();

        pr = errorR*Kp + Kd*dErrorR + Ki*sumErrorR + Kf*speedR - gyroError*Kg;
        pl = errorL*Kp + Kd*dErrorL + Ki*sumErrorL + Kf*speedL + gyroError*Kg;

        hwmap.rw1.setPower(pr);
        hwmap.rw2.setPower(pr);
        hwmap.lw1.setPower(pl);
        hwmap.lw2.setPower(pl);

        telemetry.addData("pr",pr);
        telemetry.addData("pl",pl);
        telemetry.addData("errorL:", errorL);
        telemetry.addData("errorR",errorR);


        //csvData += setL + ","+hwmap.lw1.getCurrentPosition()+ "," + setR + ","+ hwmap.lw1.getCurrentPosition() +"," + Kd*dErrorL + "," + Kf*speedL + "," + gyroError*Kg +"\r\n";

        sumErrorL += errorL;
        sumErrorR += errorR;

        if(sumErrorL > 1/Ki){
            sumErrorL = (int) (1/Ki);
        }
        if(sumErrorR > 1/Ki){
            sumErrorR = (int) (1/Ki);
        }


        previousErrorL = errorL;
        previousErrorR = errorR;


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
