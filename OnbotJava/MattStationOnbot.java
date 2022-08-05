package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Matt-Station-Onbot", group = "Station")
public class MattStationOnbot extends LinearOpMode {

  private Servo stdservo;
  private DcMotor motorL;
  private DcMotor motorR;
  private CRServo crservo;
  private ColorSensor distance_REV_ColorRangeSensor;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    int MeasuredMaxV;

    stdservo = hardwareMap.get(Servo.class, "stdservo");
    motorL = hardwareMap.get(DcMotor.class, "motorL");
    motorR = hardwareMap.get(DcMotor.class, "motorR");
    crservo = hardwareMap.get(CRServo.class, "crservo");
    distance_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "distance");

    idle();
    // Put initialization blocks here.
    MeasuredMaxV = 580;
    stdservo.scaleRange(0.1, 0.7);
    motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    motorR.setDirection(DcMotorSimple.Direction.REVERSE);
    // Put run blocks here.
    waitForStart();
    crservo.setPower(1);
    stdservo.setPosition(0);
    sleep(1000);
    stdservo.setPosition(1);
    sleep(1000);
    stdservo.setPosition(0);
    crservo.setPower(0);
    while (opModeIsActive() && (((OpticalDistanceSensor) distance_REV_ColorRangeSensor).getLightDetected() < 0.2 || ((DistanceSensor) distance_REV_ColorRangeSensor).getDistance(DistanceUnit.CM) > 8)) {
      ((DcMotorEx) motorL).setVelocity(0.3 * MeasuredMaxV);
      ((DcMotorEx) motorR).setVelocity(0.3 * MeasuredMaxV);
      telemetry.addData("range", ((DistanceSensor) distance_REV_ColorRangeSensor).getDistance(DistanceUnit.CM));
      telemetry.addData("light", ((OpticalDistanceSensor) distance_REV_ColorRangeSensor).getLightDetected());
      telemetry.addData("L position", motorL.getCurrentPosition());
      telemetry.addData("R position", motorR.getCurrentPosition());
      telemetry.addData("L V", ((DcMotorEx) motorL).getVelocity());
      telemetry.addData("R V", ((DcMotorEx) motorR).getVelocity());
      telemetry.update();
    }
    motorL.setPower(0);
    motorR.setPower(0);
    while (opModeIsActive()) {
      telemetry.addData("range", ((DistanceSensor) distance_REV_ColorRangeSensor).getDistance(DistanceUnit.CM));
      telemetry.addData("light", ((OpticalDistanceSensor) distance_REV_ColorRangeSensor).getLightDetected());
      telemetry.addData("L position", motorL.getCurrentPosition());
      telemetry.addData("R position", motorR.getCurrentPosition());
      telemetry.update();
    }
  }
}