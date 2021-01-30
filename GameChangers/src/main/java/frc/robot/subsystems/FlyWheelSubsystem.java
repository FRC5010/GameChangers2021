/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlyWheelSubsystem extends SubsystemBase {
  /**
   * Creates a new FlyWheelSubsystem.
   */
  public CANSparkMax motor;
  public CANSparkMax hood;
  public FlyWheelSubsystem(CANSparkMax m1, CANSparkMax hood) {
    motor = m1;
    this.hood = hood;
  }

  public void spinUpWheel(double d){
    motor.set(1 * d);
  }

  public void end(){
    motor.set(0);
  }

  public void moveHood(double pow){
    this.hood.set(.1 * pow);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
