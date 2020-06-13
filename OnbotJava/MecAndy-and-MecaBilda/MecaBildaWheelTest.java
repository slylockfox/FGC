package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="MecaBildaWheelTest", group="MecaBilda")
public class MecaBildaWheelTest extends LinearOpMode {

    float rotate_angle = 0;
    double reset_angle = 0;
    
    private DcMotor front_left_wheel = null;
    private DcMotor back_left_wheel = null;
    private DcMotor back_right_wheel = null;
    private DcMotor front_right_wheel = null;

    private DcMotor m0 = null;
    private DcMotor m1 = null;
    private DcMotor m2 = null;
    private DcMotor m3 = null;
    

    BNO055IMU imu;
    @Override
    public void runOpMode() {
        
        front_left_wheel = hardwareMap.dcMotor.get("lf"); m1 = front_left_wheel;
        back_left_wheel = hardwareMap.dcMotor.get("lr"); m0 = back_left_wheel;
        back_right_wheel = hardwareMap.dcMotor.get("rr"); m2 = back_right_wheel;
        front_right_wheel = hardwareMap.dcMotor.get("rf"); m3 = front_right_wheel;
        
        front_left_wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right_wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_left_wheel.setDirection(DcMotor.Direction.REVERSE); // was REVERSE
        back_left_wheel.setDirection(DcMotor.Direction.REVERSE); // was REVERSE
        front_right_wheel.setDirection(DcMotor.Direction.FORWARD); // was FORWARD
        back_right_wheel.setDirection(DcMotor.Direction.FORWARD); // was FORWARD
        
        front_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // while(!opModeIsActive()){} // like waitforstart
        
        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            // actual auto code
            
            // front left
            front_left_wheel.setPower(0.5);
            front_left_wheel.setTargetPosition(2000);
            front_left_wheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && front_left_wheel.isBusy()) {
                telemetry.addData("FL position", front_left_wheel.getCurrentPosition());
                telemetry.update();
                sleep(100);
            }
            
            // front right
            front_right_wheel.setPower(0.5);
            front_right_wheel.setTargetPosition(2000);
            front_right_wheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && front_right_wheel.isBusy()) {
                telemetry.addData("FR position", front_right_wheel.getCurrentPosition());
                telemetry.update();
                sleep(100);
            }

            // back left
            back_left_wheel.setPower(0.5);
            back_left_wheel.setTargetPosition(2000);
            back_left_wheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && back_left_wheel.isBusy()) {
                telemetry.addData("FR position", back_left_wheel.getCurrentPosition());
                telemetry.update();
                sleep(100);
            }

            // back right
            back_right_wheel.setPower(0.5);
            back_right_wheel.setTargetPosition(2000);
            back_right_wheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && back_right_wheel.isBusy()) {
                telemetry.addData("FR position", back_right_wheel.getCurrentPosition());
                telemetry.update();
                sleep(100);
            }

        }
    }
    
}
    
