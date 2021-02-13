// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private CANSparkMax m9;
  private CANSparkMax m12;

  public IntakeSubsystem(CANSparkMax m9, CANSparkMax m12) { 
    this.m9 = m9;
    this.m12 = m12;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spin(double power){
    m12.set(power);
  }

  public void moveIntake(double power){
    m9.set(power);
  }

  public void stop(){
    m12.set(0);
  }
}
