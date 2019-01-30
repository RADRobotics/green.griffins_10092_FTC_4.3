package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "ThomasMotoinProfileTest", group = "Competition")
public class ThomasMotionProfileTest extends OpMode {
    private DcMotor armExtendRight;
    //arm extend right
    private DcMotor armExtendLeft;
    //arm extend left

    public ElapsedTime mRuntime = new ElapsedTime();

    @Override
    public void init() {

        armExtendRight = hardwareMap.dcMotor.get("armExtend");
        armExtendLeft = hardwareMap.dcMotor.get("armExtend2");

        armExtendLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtendRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armExtendLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armExtendRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armExtendRight.setDirection(DcMotor.Direction.REVERSE);

        mRuntime.reset();
    }

    private int previousError = 0;
    private int sumError = 0;
    private double maxSpeed = 0;
    private double moveStartTime = 0;

    @Override
    public void loop() {
        int setPoint = 300;
        double extendPower = 0;

        int error = setPoint - getArmExtendEncoder();


        if(gamepad1.a){
            /*
            double Kp = 0.01;
            double Kd = 0.01;
            double Ki = 0.00005;

            double currentExtendSpeed = (previousError-error)/(mRuntime.time()+0.000000001);

            if(currentExtendSpeed>maxSpeed){
                maxSpeed = currentExtendSpeed;
            }

            extendPower = Kp * error + Ki * sumError + Kd * currentExtendSpeed;

            setArmExtendPower(extendPower);

            //Storing previous error and summing error for the integral
            previousError = error;
            sumError += error;

            //Integral saturatoin
            if(sumError*Ki>0.5){
                sumError = (int)(0.5/Ki);
            }
            */

            double speed = getMotoinProfileSpeed(5, 10, 300, (mRuntime.time()-moveStartTime));
            double currentExtendSpeed = (previousError-error)/(mRuntime.time()+0.000000001);

            double speedError = speed - currentExtendSpeed;

            extendPower = (1.0/20.0)*speed + speedError*0.1;
            setArmExtendPower(extendPower);

            telemetry.addData("set point speed",speed);
            telemetry.addData("Current speed",currentExtendSpeed);
        }else{
            moveStartTime = mRuntime.time();
            setArmExtendPower(0);

        }

        telemetry.addData("Combined encoder", getArmExtendEncoder());
        telemetry.addData("Power", extendPower);
        telemetry.addData("Max speed", maxSpeed);


    }

    public void setArmExtendPower(double power){
        armExtendRight.setPower(-power);
        armExtendLeft.setPower(-power);
    }

    public int getArmExtendEncoder (){
        return ((armExtendLeft.getCurrentPosition() + armExtendRight.getCurrentPosition())/2);
    }

    public double getMotoinProfileSpeed(double maxVel, double maxAcc, double distance, double t){
        //Accelleratoin time
        double Ta = maxVel/maxAcc;

        //Cruise time
        double Tc = (distance - Ta*maxVel)/maxVel;

        telemetry.addData("Elapsed move time",t);
        telemetry.addData("Ta", Ta);
        telemetry.addData("Tc",Tc);

        if(t < Ta){
            return t*maxAcc;
        }else if(Ta <= t && t <= (Ta+Tc)){
            return maxVel;
        }else if(t > (Ta+Tc)){
            return (2*Ta + Tc - t)*maxAcc;
        }else{
            return 0.1;
        }
    }
}


