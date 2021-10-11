// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.mechanisms.ShooterConstants;
import frc.robot.subsystems.FlyWheelSubsystem;

public class TriangleHood extends CommandBase {
  /** Creates a new TriangleHood. */
  FlyWheelSubsystem flyWheelSubsystem;
  public TriangleHood(FlyWheelSubsystem flyWheelSubsystem) {
    this.flyWheelSubsystem = flyWheelSubsystem;
    addRequirements(flyWheelSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    flyWheelSubsystem.PIDHood(ShooterConstants.triShoot);
    // 4 y button (manuel up) == 2014 on hood
    //"4 y buttons from the bottom" -Riley on if the petentiometer breaks and we need to reset the hood value
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flyWheelSubsystem.moveHood(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
