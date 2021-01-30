/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlyWheelSubsystem;

public class ShootBall extends CommandBase {
  /**
   * Creates a new ShootBall.
   */
  public FlyWheelSubsystem flyWheelSubsystem;
  Joystick joystick;
  public ShootBall(FlyWheelSubsystem flyWheelSubsystem, Joystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.joystick = joystick;
    this.flyWheelSubsystem = flyWheelSubsystem;
    addRequirements(this.flyWheelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    flyWheelSubsystem.spinUpWheel(joystick.getRawAxis(3));
    flyWheelSubsystem.moveHood(joystick.getRawAxis(5));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flyWheelSubsystem.end();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
