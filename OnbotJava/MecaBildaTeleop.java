// My fork: https://github.com/slylockfox/FGC_Guides/blob/master/driveJava.java

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Disabled
@TeleOp(name="MecaBildaTeleop", group="MecaBilda")
public class MecaBildaTeleop extends LinearOpMode {
    // private Gyroscope imu;

    private DcMotor lift;
    int LiftPos;
    boolean IncreaseLiftPos;
    boolean DecreaseLiftPos;
    double LiftStepCount;
    double LiftMaxCount;
    
    private Servo arm;
    private Servo grip;

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
    
    private Servo liftBase;
    private Servo liftLifter;
    
    BNO055IMU imu;
    @Override
    public void runOpMode() {
        
        front_left_wheel = hardwareMap.dcMotor.get("lf"); m3 = front_left_wheel;
        back_left_wheel = hardwareMap.dcMotor.get("lr"); m2 = back_left_wheel;
        back_right_wheel = hardwareMap.dcMotor.get("rr"); m1 = back_right_wheel;
        front_right_wheel = hardwareMap.dcMotor.get("rf"); m0 = front_right_wheel;
        
        front_left_wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        back_left_wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        front_right_wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        back_right_wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

        front_left_wheel.setDirection(DcMotor.Direction.FORWARD); // was REVERSE
        back_left_wheel.setDirection(DcMotor.Direction.FORWARD); // was REVERSE
        front_right_wheel.setDirection(DcMotor.Direction.REVERSE); // was FORWARD
        back_right_wheel.setDirection(DcMotor.Direction.REVERSE); // was FORWARD
        
        front_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift = hardwareMap.dcMotor.get("lift");
        arm = hardwareMap.servo.get("arm");
        grip = hardwareMap.servo.get("grip");
        
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // remap axes in order to  use with vertically mounted REV hub
        
        byte AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes
        byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis
        //Need to be in CONFIG mode to write to registers
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
        sleep(100); //Changing modes requires a delay before doing anything else
        //Write to the AXIS_MAP_CONFIG register
        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG,AXIS_MAP_CONFIG_BYTE & 0x0F);
        //Write to the AXIS_MAP_SIGN register
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_MAP_SIGN_BYTE & 0x0F);
        //Need to change back into the IMU mode to use the gyro
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal & 0x0F);
        sleep(100); //Changing modes again requires a delay
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu.initialize(parameters);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setPower(0.5);
        LiftStepCount = 500;
        LiftMaxCount = 5000;
        LiftPos = 0;
        DecreaseLiftPos = false;
        IncreaseLiftPos = false;

        grip.setPosition(0.2);
        arm.setPosition(0.9);

        while(!opModeIsActive() && !isStopRequested()){} // like waitforstart

        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        while(opModeIsActive()){
                lift();            
                grip();
                wrist();
                
                drive();
                resetAngle();
                //driveSimple();
                
                telemetry.update();
        }
    }
    
    public void driveSimple(){
        double power = 0.5;
        if(gamepad1.dpad_up){ //Forward
            front_left_wheel.setPower(-power);
            back_left_wheel.setPower(-power);
            back_right_wheel.setPower(-power);
            front_right_wheel.setPower(-power);
        }
        else if(gamepad1.dpad_left){ //Left
            front_left_wheel.setPower(power);
            back_left_wheel.setPower(-power);
            back_right_wheel.setPower(power);
            front_right_wheel.setPower(-power);
        }
        else if(gamepad1.dpad_down){ //Back
            front_left_wheel.setPower(power);
            back_left_wheel.setPower(power);
            back_right_wheel.setPower(power);
            front_right_wheel.setPower(power);
        }
        else if(gamepad1.dpad_right){ //Right
            front_left_wheel.setPower(-power);
            back_left_wheel.setPower(power);
            back_right_wheel.setPower(-power);
            front_right_wheel.setPower(power);
        }
        else if(Math.abs(gamepad1.right_stick_x) > 0){ //Rotation
            front_left_wheel.setPower(-gamepad1.right_stick_x);
            back_left_wheel.setPower(-gamepad1.right_stick_x);
            back_right_wheel.setPower(gamepad1.right_stick_x);
            front_right_wheel.setPower(gamepad1.right_stick_x);
        }
        else{
            front_left_wheel.setPower(0);
            back_left_wheel.setPower(0);
            back_right_wheel.setPower(0);
            front_right_wheel.setPower(0);
        }
    }
    
    public void wrist() {
         if (gamepad1.x) {
            arm.setPosition(0.7);
         } else if (gamepad1.y){
            arm.setPosition(0.5);
         } else if (gamepad1.b) {
            arm.setPosition(0.15);
         }
    }

    public void grip() {
        double grippos;
        grippos = grip.getPosition();
        if (gamepad1.right_bumper) {
          grippos = 0.2; // open
        } else if (gamepad1.right_trigger > 0.5) {
          grippos = 0.32; // closed
        }
        grip.setPosition(grippos);
    }
    
    public void lift() {
        if (gamepad1.left_bumper) {
          IncreaseLiftPos = true;
        } else if (gamepad1.left_trigger > 0.5) {
          DecreaseLiftPos = true;
        } else if (IncreaseLiftPos && LiftPos < LiftMaxCount) {
          LiftPos = LiftPos + 500;
          IncreaseLiftPos = false;
        } else if (DecreaseLiftPos && LiftPos > 0) {
          LiftPos = LiftPos - 500;
          DecreaseLiftPos = false;
        }
        lift.setTargetPosition(LiftPos);
        telemetry.addData("lift pos", lift.getCurrentPosition());
        telemetry.addData("lift target", LiftPos);
    }
    
    public void drive() {
        double drivescale = 1.5;
        double rotatescale = 2;
        double Protate = gamepad1.right_stick_x/rotatescale;
        double stick_x = gamepad1.left_stick_x * Math.sqrt(Math.pow(1-Math.abs(Protate), 2)/drivescale); //Accounts for Protate when limiting magnitude to be less than 1
        double stick_y = gamepad1.left_stick_y * Math.sqrt(Math.pow(1-Math.abs(Protate), 2)/drivescale);
        double theta = 0;
        double Px = 0;
        double Py = 0;

        double gyroAngle = getHeading() * Math.PI / 180; //Converts gyroAngle into radians
        if (gyroAngle <= 0) { 
            gyroAngle = gyroAngle + (Math.PI / 2);
        } else if (0 < gyroAngle && gyroAngle < Math.PI / 2) {
            gyroAngle = gyroAngle + (Math.PI / 2);
        } else if (Math.PI / 2 <= gyroAngle) {
            gyroAngle = gyroAngle - (3 * Math.PI / 2);
        }
        gyroAngle = -1 * gyroAngle;
        
        if(gamepad1.right_bumper){ //Disables gyro, sets to -Math.PI/2 so front is defined correctly. 
            gyroAngle = -Math.PI/2;
        }
        
        //Linear directions in case you want to do straight lines.
        if(gamepad1.dpad_right){
            stick_x = 0.7;
        }
        else if(gamepad1.dpad_left){
            stick_x = -0.7;
        }
        if(gamepad1.dpad_up){
            stick_y = -0.7;
        }
        else if(gamepad1.dpad_down){
            stick_y = 0.7;
        }
        
        
        //MOVEMENT
        theta = Math.atan2(stick_y, stick_x) - gyroAngle - (Math.PI / 2);
        Px = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta + Math.PI / 4));
        Py = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta - Math.PI / 4));
        
        telemetry.addData("Gyro", getHeading());

        telemetry.addData("Stick_X", stick_x);
        telemetry.addData("Stick_Y", stick_y);
        telemetry.addData("Magnitude",  Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)));
        telemetry.addData("Front Left", Py - Protate);
        telemetry.addData("Back Left", Px - Protate);
        telemetry.addData("Back Right", Py + Protate);
        telemetry.addData("Front Right", Px + Protate);
        
        // to make sure encoder wires are connected
        telemetry.addData("m0 position", m0.getCurrentPosition());
        telemetry.addData("m1 position", m1.getCurrentPosition());
        telemetry.addData("m2 position", m2.getCurrentPosition());
        telemetry.addData("m3 position", m3.getCurrentPosition());
        
        front_left_wheel.setPower(Py - Protate);
        back_left_wheel.setPower(Px - Protate);
        back_right_wheel.setPower(Py + Protate);
        front_right_wheel.setPower(Px + Protate);
    }
    public void resetAngle(){
        if(gamepad1.a){
            reset_angle = getHeading() + reset_angle;
        }
    }
    public double getHeading(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        if(heading < -180) {
            heading = heading + 360;
        }
        else if(heading > 180){
            heading = heading - 360;
        }
        heading = heading - reset_angle;
        return heading;
    }
}
    
