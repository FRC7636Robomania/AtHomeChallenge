// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Constants.Path;
import frc.robot.commands.auto.TrajectoryCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.chassis.trajectory.TrajectoryFactory;
import frc.robot.subsystems.chassis.trajectory.TrajectorySystem;

public class CheckBall extends CommandBase {
  /** Creates a new CheckBall. */
  TrajectorySystem drivetrain;
  Intake intake;
  public CheckBall(TrajectorySystem drivetrain, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.isBall = Vision.getBall();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Constants.next = new InstantCommand(()->intake.intakeForward(), intake);
    if(Constants.isBall){
      Constants.next = TrajectoryCommand.build(drivetrain, TrajectoryCommand.OutputMode.VOLTAGE, drivetrain, TrajectoryFactory.getTrajectory(Path.B_Blue[0]));
      // Constants.next.andThen(TrajectoryCommand.build(drivetrain, TrajectoryCommand.OutputMode.VOLTAGE, drivetrain, TrajectoryFactory.getTrajectory(Path.B_Blue[0])));
      SmartDashboard.putString("path", Path.B_Blue[0]);
    }else{
      Constants.next = TrajectoryCommand.build(drivetrain, TrajectoryCommand.OutputMode.VOLTAGE, drivetrain, TrajectoryFactory.getTrajectory(Path.A_Blue[0]));
      // Constants.next.andThen(TrajectoryCommand.build(drivetrain, TrajectoryCommand.OutputMode.VOLTAGE, drivetrain, TrajectoryFactory.getTrajectory(Path.A_Blue[0])));
      SmartDashboard.putString("path", Path.A_Blue[0]);
    }
    // Constants.next.beforeStarting(()->intake.intakeForward(), intake);
    
    // Constants.next.andThen(new InstantCommand(()->intake.stop(), intake));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Constants.isFinish = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
