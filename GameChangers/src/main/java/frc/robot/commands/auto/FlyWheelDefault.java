// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlyWheelSubsystem;

public class FlyWheelDefault extends CommandBase {
  /** Creates a new FlyWheelDefault. */
  private FlyWheelSubsystem flyWheelSubsystem;
  private Joystick driver;
  public FlyWheelDefault(Joystick driver, FlyWheelSubsystem flyWheelSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.flyWheelSubsystem = flyWheelSubsystem;
    this.driver = driver;
    addRequirements(flyWheelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    flyWheelSubsystem.setWheelSpeed(driver.getRawAxis(2));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flyWheelSubsystem.setWheelSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
