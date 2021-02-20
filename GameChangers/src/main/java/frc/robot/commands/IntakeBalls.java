// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ControlConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeBalls extends CommandBase {
  /** Creates a new IntakeBalls. */
  private Joystick joystick;
  private IntakeSubsystem intakeSubsystem;
  private double power;

  public IntakeBalls(IntakeSubsystem intakeSubsystem, Joystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    this.joystick = joystick;
    addRequirements(intakeSubsystem);
  }

  public IntakeBalls(IntakeSubsystem intakeSubsystem, double power){
    this.intakeSubsystem = intakeSubsystem;
    this.power = power;
    addRequirements(intakeSubsystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(joystick != null){
      intakeSubsystem.spin(joystick.getRawAxis(ControlConstants.intakeAxis) - joystick.getRawAxis(ControlConstants.outtakeAxis));

    }else{
      intakeSubsystem.spin(power);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
