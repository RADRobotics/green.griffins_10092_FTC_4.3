package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "CompetitionRobotDriveBase", group = "Competition")
@Disabled
public class CompetitionRobotDriveBase extends OpMode {
    private DcMotor leftWheelFront;
    private DcMotor leftWheelBack;
    private DcMotor rightWheelFront;
    private DcMotor rightWheelBack;
    private double nitro;

    @Override
    public void init() {
        leftWheelFront = hardwareMap.dcMotor.get("leftWheelFront");
        leftWheelBack = hardwareMap.dcMotor.get("leftWheelBack");
        rightWheelFront = hardwareMap.dcMotor.get("rightWheelFront");
        rightWheelBack = hardwareMap.dcMotor.get("rightWheelBack");

        nitro = 0;

        leftWheelFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWheelBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheelFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheelBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftWheelFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftWheelBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheelFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheelBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    @Override
    public void loop() {
        nitro= 1-gamepad1.right_trigger*.8;

        rightWheelFront.setPower((-gamepad1.left_stick_y-gamepad1.right_stick_x)*nitro);
        rightWheelBack.setPower((-gamepad1.left_stick_y-gamepad1.right_stick_x)*nitro);
        leftWheelBack.setPower((gamepad1.left_stick_y-gamepad1.right_stick_x)*nitro);
        leftWheelFront.setPower((gamepad1.left_stick_y-gamepad1.right_stick_x)*nitro);
    }
}


