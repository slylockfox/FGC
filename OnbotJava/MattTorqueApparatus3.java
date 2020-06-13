package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "MattTorqueApparatus3", group = "test")
public class MattTorqueApparatus3 extends LinearOpMode {

  private DcMotorEx motor0;
  private CRServo falcon500;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    ElapsedTime timer1;
    ElapsedTime timer2;
    double falconSpeed;
    double rpm, v_counts;

    motor0 = hardwareMap.get(DcMotorEx.class, "motor0");
    falcon500 = hardwareMap.crservo.get("falcon500");

    // Put initialization blocks here.
    motor0.setDirection(DcMotorSimple.Direction.REVERSE);
    motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    falcon500.setPower(0);
    timer1 = new ElapsedTime();
    timer2 = new ElapsedTime();
    falconSpeed = 0;
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      timer1.reset();
      timer2.reset();
      rpm = 0.0;
      while (rpm < 3000 && timer1.time() < 10 && opModeIsActive()) {
        // Wind up for 10 secs
        falcon500.setPower(falconSpeed);
        if (timer2.time() > 0.5) {
          falconSpeed = falconSpeed + 0.02;
          timer2.reset();
        }
        v_counts = motor0.getVelocity();
        rpm = v_counts / 2;
        telemetry.addData("PWM", falconSpeed);
        telemetry.addData("encoder", motor0.getCurrentPosition());
        telemetry.addData("V_counts", v_counts);
        telemetry.addData("RPM", rpm);
        telemetry.update();
      }
      timer1.reset();
      motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      while (timer1.time() < 10 && opModeIsActive()) {
        // count encoders for 10 Secs
        v_counts = motor0.getVelocity();
        rpm = v_counts / 2;
        telemetry.addData("PWM", falconSpeed);
        telemetry.addData("encoder", motor0.getCurrentPosition());
        telemetry.addData("V_counts", v_counts);
        telemetry.addData("RPM", rpm);
        telemetry.update();
      }
      timer1.reset();
      timer2.reset();
      while (timer1.time() < 10 && opModeIsActive()) {
        // Wind down for 10 secs
        if (falconSpeed > 0) { falcon500.setPower(falconSpeed);}
        if (timer2.time() > 1) {
          falconSpeed = falconSpeed - 0.04;
          timer2.reset();
        }
      }
      sleep(10000);
    }
  }
}
