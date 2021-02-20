/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperOmniSubsystem extends SubsystemBase {
  /**
   * Creates a new HopperOmniSubsystem.
   */
  public CANSparkMax HOmniMotor;
  public CANSparkMax hopperMotor;
  public HopperOmniSubsystem(CANSparkMax HOmniMotor, CANSparkMax hopperMotor) {
    this.HOmniMotor = HOmniMotor;

    this.hopperMotor = hopperMotor;

  }
  public void SetOmniSpeed(double speed){
      HOmniMotor.set(speed);
  }

  public void SetHopperSpeed(double speed){
    hopperMotor.set(speed);
  }
  public void end(){
    HOmniMotor.set(0);
    hopperMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
