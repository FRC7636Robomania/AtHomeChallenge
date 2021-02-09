// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  private static NetworkTable table = NetworkTableInstance.getDefault().getTable(Constants.visionName);
  /** Creates a new Vision. */
  private Vision() {}

  /**
   * Return true if the area bigger than 0
   * @return
   */
  public static boolean getBall(){
    return getArea() > 0;
  }
  
  public static boolean isRight(){
    double x = 0;
    for(int i = 0; i < 3; i++){
      x += table.getEntry("x").getDouble(0.0);
    }
    SmartDashboard.putBoolean("isRight", (x / 3) > 80);
    return (x / 3) > 80;
  }

  public static double getArea(){
    double a = 0;
    for(int i = 0; i < 3; i++){
      a += table.getEntry("area").getDouble(0.0);
    }
    return a / 3;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
