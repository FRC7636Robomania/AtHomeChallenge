// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Path;
import frc.robot.commands.auto.TrajectoryCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.chassis.trajectory.TrajectoryFactory;
import frc.robot.subsystems.chassis.trajectory.TrajectorySystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CheckBlue extends SequentialCommandGroup {
  /** Creates a new CheckBlue. */
  public CheckBlue(TrajectorySystem drivetrain, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new InstantCommand(()->intake.forward()));
    addCommands(new CheckBall());
    if(Constants.isBall){
      addCommands(TrajectoryCommand.build(drivetrain, TrajectoryCommand.OutputMode.VOLTAGE, drivetrain, TrajectoryFactory.getTrajectory(Path.B_Blue[0])));
      SmartDashboard.putString("path", Path.B_Blue[0]);
      SmartDashboard.putBoolean("checkBall", true);
    }else{
      addCommands(TrajectoryCommand.build(drivetrain, TrajectoryCommand.OutputMode.VOLTAGE, drivetrain, TrajectoryFactory.getTrajectory(Path.A_Blue[0])));
      SmartDashboard.putString("path", Path.A_Blue[0]);
      SmartDashboard.putBoolean("checkBall", false);
    }
    addCommands(new InstantCommand(()->intake.stop()));
  }
}
