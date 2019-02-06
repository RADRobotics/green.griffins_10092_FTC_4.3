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

package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class hmap
{
    /* Public OpMode members. */
    public gyroCompass gyro = null;

    public DcMotor lw1= null;
    public DcMotor lw2 = null;
    public DcMotor rw1 = null;
    public DcMotor rw2 = null;

    public DcMotor rightArm = null;
    public DcMotor leftArm = null;
    public DcMotor armExtendRight = null;
    public DcMotor armExtendLeft = null;

    public Servo rightLock = null;
    public Servo leftLock = null;
    public Servo intake = null;
    public Servo intake2 = null;

    /* local OpMode members. */
    private HardwareMap hwMap           =  null;
    public ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public hmap(){

    }
    public void rs(double power){
        rw1.setPower(power);
        rw2.setPower(power);
    }
    public void ls(double power){
        lw1.setPower(power);
        lw2.setPower(power);
    }
    public void arm(double power){

        int armError=0;//rightArm.getCurrentPosition()-leftArm.getCurrentPosition();
        double Ka=.002;

        rightArm.setPower(power-armError*Ka);
        leftArm.setPower(power+armError*Ka);
    }
    public void zero(){
        lw1.setPower(0);
        lw2.setPower(0);
        rw1.setPower(0);
        rw2.setPower(0);
        rightArm.setPower(0);
        leftArm.setPower(0);
        armExtendLeft.setPower(0);
        armExtendRight.setPower(0);
    }
    public String print(){
        return "rw: " + rw1.getCurrentPosition() +" lw:" + lw1.getCurrentPosition() + "\n lArm:" + leftArm.getCurrentPosition() + ", rArm: "+ rightArm.getCurrentPosition()+",  armExtL: " + armExtendLeft.getCurrentPosition() + "armExtR: " + armExtendRight.getCurrentPosition();
    }
    public void lock(boolean state){
        if(state){
            leftLock.setPosition(.25);
            rightLock.setPosition(.4);
        }
        else{
            leftLock.setPosition(0.37);
            rightLock.setPosition(.32);
        }
    }
    public void reset(){
        armExtendLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtendRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtendLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armExtendRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rw1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rw1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lw1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lw1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    /* Initialize standard Hardware interfaces */
    public void initGyro(){
        gyro = new gyroCompass(hwMap);
    }
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;

        rw1 = hwMap.dcMotor.get("rw");
        lw1 = hwMap.dcMotor.get("lw");
        lw2 = hwMap.dcMotor.get("lw2");
        rw2 = hwMap.dcMotor.get("rw2");

        rightArm = hwMap.dcMotor.get("rightArm");
        leftArm = hwMap.dcMotor.get("leftArm");
        armExtendRight = hwMap.dcMotor.get("armExtend");
        armExtendLeft = hwMap.dcMotor.get("armExtend2");

        rightLock = hwMap.servo.get("rightLock");
        leftLock = hwMap.servo.get("leftLock");
        intake = hwMap.servo.get("intake");
        intake2 = hwMap.servo.get("intake2");

        rw1.setDirection(DcMotorSimple.Direction.REVERSE);
        rw2.setDirection(DcMotorSimple.Direction.REVERSE);
        armExtendRight.setDirection(DcMotorSimple.Direction.REVERSE);
        leftArm.setDirection(DcMotorSimple.Direction.REVERSE);

        rw1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rw2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lw1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lw2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtendRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtendLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armExtendLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armExtendRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        rw1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rw2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lw1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lw2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
 }

