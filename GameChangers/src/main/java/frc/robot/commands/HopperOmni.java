/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ControlConstants;
import frc.robot.subsystems.HopperOmniSubsystem;

public class HopperOmni extends CommandBase {
  public HopperOmniSubsystem hopperOmniSubsystem;
  private Joystick operator;

  /**
   * Creates a new HopperOmni.
   */
  public HopperOmni(HopperOmniSubsystem hopperOmniSubsystem, Joystick operator) {
    this.operator = operator;
    this.hopperOmniSubsystem = hopperOmniSubsystem;
    addRequirements(hopperOmniSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hopperOmniSubsystem.SetHopperSpeed(operator.getRawAxis(ControlConstants.operatorLeftY)*.4);
    hopperOmniSubsystem.SetOmniSpeed(operator.getRawAxis(ControlConstants.operatorRightY));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopperOmniSubsystem.end();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
