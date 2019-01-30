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

import android.content.Context;
import android.media.AudioManager;
import android.media.MediaPlayer;
import android.media.SoundPool;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.OutputStreamWriter;
import java.util.ArrayList;

import org.firstinspires.ftc.teamcode.utils.gyroCompass;
/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="soundTest", group="Iterative Opmode")

public class soundTest extends OpMode
{
    // Declare OpMode members.
int intake=0;
    gyroCompass gyro;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime runtime2 = new ElapsedTime();
    private DcMotor rw = null;
    private DcMotor lw = null;
    DcMotor arm = null;
    DcMotor armExtend;
    public SoundPool mySound;
    public int beepID;
    ArrayList<String> list = new ArrayList<String>();
    String marks[][]={{"hi","bu"},{"bap","boop"}};

    String thing = "";
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        armExtend = hardwareMap.dcMotor.get("armExtend2");

        arm = hardwareMap.dcMotor.get("leftArm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        gyro = new gyroCompass(hardwareMap);

        rw = hardwareMap.dcMotor.get("rightWheel");
        lw = hardwareMap.dcMotor.get("leftWheel");

        rw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mySound= new SoundPool(1,AudioManager.STREAM_MUSIC,0);
        beepID = mySound.load(hardwareMap.appContext, R.raw.mission_nightwing,1);

        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        //rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
       // leftDrive.setDirection(DcMotor.Direction.FORWARD);
        //rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */

    public void start() {
            mySound.play(beepID,1,1,1,1,1);
            runtime.reset();
            runtime2.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if(runtime.seconds()>.025){
            //list.add(runtime2.seconds() + " " + rw.getCurrentPosition() + " " + lw.getCurrentPosition());
            //thing += "s:"+runtime2.seconds() + " r:" + rw.getCurrentPosition() + " l:" + lw.getCurrentPosition() + "; ";
            if(gamepad2.right_bumper){
                intake=1;
            }
            else{
                intake=0;
            }
            thing += "{"+rw.getCurrentPosition()+","+lw.getCurrentPosition()+","+ (int)(gyro.getHeading()*1000)+","+arm.getCurrentPosition()+","+armExtend.getCurrentPosition()+","+intake +"},";
            telemetry.addData("sent", "");
            runtime.reset();
        }

//        if(gamepad1.a){
//            mySound.play(beepID,1,1,1,0,1);
//        }
//        // Setup a variable for each drive wheel to save power level for telemetry
//        telemetry.addData("encoders", rw.getCurrentPosition() +" " + lw.getCurrentPosition());
//        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        String filename = "data.csv";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, thing);
        telemetry.log().add("saved to '%s'", filename);
    }

}
