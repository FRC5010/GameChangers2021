// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.mechanisms.Drive;
import frc.robot.mechanisms.DriveConstants;

public class SwitchDriveDirection extends CommandBase {
  /** Creates a new SwitchDriveDirection. */
  private Joystick driver;
  private int time;
  public SwitchDriveDirection(Joystick driver) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driver = driver;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveConstants.driveInversion *= -1;
    driver.setRumble(RumbleType.kLeftRumble, 1);
    driver.setRumble(RumbleType.kRightRumble, 1);
    time = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driver.setRumble(RumbleType.kLeftRumble, 0);
    driver.setRumble(RumbleType.kRightRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return time++ > 10;
  }
}
