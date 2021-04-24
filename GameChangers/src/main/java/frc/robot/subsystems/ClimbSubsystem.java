// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  private CANSparkMax arm1,arm2;

  public ClimbSubsystem(CANSparkMax arm1, CANSparkMax arm2) {
    this.arm1 = arm1;
    this.arm2 = arm2;
  }

  public void spinArmMotors(double left, double right){
    if(Math.abs(left) > .25){
      arm1.set(left);
    }else{
      arm1.set(0);
    }
    if(Math.abs(right) > .25){
      arm2.set(right);
    }else{
      arm2.set(0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
