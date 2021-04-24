// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ControlConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.HopperOmniSubsystem;

public class SepArmClimb extends CommandBase {
  /** Creates a new SepArmClimb. */
  Joystick operator;
  ClimbSubsystem sub;
  public SepArmClimb(Joystick operator, ClimbSubsystem subsystem, HopperOmniSubsystem hopperOmni) {
    this.operator = operator;
    sub = subsystem;
    addRequirements(sub, hopperOmni);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sub.spinArmMotors(operator.getRawAxis(ControlConstants.operatorLeftY), operator.getRawAxis(ControlConstants.operatorRightY));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sub.spinArmMotors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
