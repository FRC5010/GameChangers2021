// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeAndSpin extends CommandBase {
  /** Creates a new AutoIntakeAndSpin. */
  private IntakeSubsystem intakeSubsystem;

  public AutoIntakeAndSpin(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
    // Use addRequirements(intake) here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.moveIntake(-1);
    intakeSubsystem.spin(.75);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double encoderVal = intakeSubsystem.getIntakePosition();
    if(encoderVal <= -60){
      intakeSubsystem.moveIntake(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.moveIntake(0);
    intakeSubsystem.spin(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
