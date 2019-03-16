package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "omnibotlifttest (Blocks to Java)", group = "test")
public class omnibotlifttest extends LinearOpMode {

  private Servo liftBase;
  private Servo liftLifter;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    ElapsedTime liftBaseTimer;
    ElapsedTime liftLifterTimer;
    double liftBaseTarget;
    double liftLifterTarget;

    liftBase = hardwareMap.servo.get("liftBase");
    liftLifter = hardwareMap.servo.get("liftLifter");

    // Put initialization blocks here.
    liftBaseTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    liftLifterTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    liftBaseTarget = 0.6;
    liftLifterTarget = 0.3;
    liftBaseTimer.reset();
    liftLifterTimer.reset();
    waitForStart();
    // Put run blocks here.
    while (opModeIsActive()) {
      // Put loop blocks here.
      if (gamepad1.dpad_up) {
        liftBaseTarget = 0.5;
        liftLifterTarget = 0.68;
      } else if (gamepad1.dpad_down) {
        liftBaseTarget = 0.55;
        liftLifterTarget = 0.4;
      } else if (gamepad1.b) {
        liftBaseTarget = 0.6;
        liftLifterTarget = 0.3;
      } else {
        if (gamepad1.left_stick_y < -0.2 && liftLifterTimer.time() > 100) {
          liftLifterTimer.reset();
          liftLifterTarget = liftLifterTarget + 0.01;
        } else if (gamepad1.left_stick_y > 0.2 && liftLifterTimer.time() > 100) {
          liftLifterTimer.reset();
          liftLifterTarget = liftLifterTarget - 0.01;
        } else if (gamepad1.left_stick_x < -0.2 && liftBaseTimer.time() > 500) {
          liftBaseTimer.reset();
          liftBaseTarget = liftBaseTarget + 0.01;
        } else if (gamepad1.left_stick_x > 0.2 && liftBaseTimer.time() > 500) {
          liftBaseTimer.reset();
          liftBaseTarget = liftBaseTarget - 0.01;
        } else {
        }
      }
      liftBase.setPosition(liftBaseTarget);
      liftLifter.setPosition(liftLifterTarget);
      telemetry.update();
    }
  }
}
