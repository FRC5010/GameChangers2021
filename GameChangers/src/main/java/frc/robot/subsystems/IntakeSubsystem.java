// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.ToggleIntake;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private Joystick operator;
  private CANSparkMax m9;
  private CANSparkMax m11;
  private CANSparkMax m12;

  public IntakeSubsystem(CANSparkMax m9, CANSparkMax m11, CANSparkMax m12, Joystick operator) { 
    this.operator = operator;
    this.m9 = m9;
    this.m11 = m11;
    this.m12 = m12;
    setDefaultCommand(new IntakeBalls(this, operator));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void toggleIntake(){
    m9.set(operator.getRawAxis(5));
  }
  
  public void spin(double power){
    m12.set(power);
  }

  public void stop(){
    m12.set(0);
  }
}
