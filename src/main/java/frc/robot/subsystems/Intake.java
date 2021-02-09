// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Intake extends Spinable{
  VictorSPX intake = new VictorSPX(7);
  VictorSPX middle = new VictorSPX(1);
  VictorSPX wing   = new VictorSPX(2);
  /** Creates a new Intake. */
  public Intake() {
    intake.configFactoryDefault();
    intake.configPeakOutputForward(0.9);
    intake.configPeakOutputReverse(-0.9);
    intake.configNominalOutputForward(0.6);
    intake.configNominalOutputReverse(-0.6);
    middle.setInverted(true);
    wing.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void forward() {
    intake.set(ControlMode.PercentOutput, 0.85);
    middle.set(ControlMode.PercentOutput, 0.4);
    wing.set(ControlMode.PercentOutput, 0.35);
  }

  @Override
  public void stop() {
    intake.set(ControlMode.PercentOutput, 0);
    middle.set(ControlMode.PercentOutput, 0);
    wing.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void reverse() {
    intake.set(ControlMode.PercentOutput, -0.85);
    middle.set(ControlMode.PercentOutput, -0.4);
    wing.set(ControlMode.PercentOutput, -0.35);
  }
}
