// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Intake extends Spinable{
  VictorSPX intake = new VictorSPX(3);
  /** Creates a new Intake. */
  public Intake() {
    intake.configFactoryDefault();
    intake.configPeakOutputForward(0.95);
    intake.configPeakOutputReverse(-0.95);
    intake.configNominalOutputForward(0.7);
    intake.configNominalOutputReverse(-0.7);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void forward() {
    intake.set(ControlMode.PercentOutput, 0.85);
  }

  @Override
  public void stop() {
    intake.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void reverse() {
    intake.set(ControlMode.PercentOutput, -0.85);
  }
}
