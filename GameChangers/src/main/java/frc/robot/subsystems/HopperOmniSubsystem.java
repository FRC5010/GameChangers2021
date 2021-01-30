/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.HopperOmni;

public class HopperOmniSubsystem extends SubsystemBase {
  /**
   * Creates a new HopperOmniSubsystem.
   */
  public Joystick driver;
  public CANSparkMax HOmniMotor;
  public CANSparkMax hopperMotor;
  public HopperOmniSubsystem(CANSparkMax HOmniMotor, Joystick driver, CANSparkMax hopperMotor) {
    this.HOmniMotor = HOmniMotor;
    this.driver = driver;
    this.hopperMotor = hopperMotor;
    setDefaultCommand(new HopperOmni(this));
  }
  public void SetOmniSpeed(){
    double currSpeed = driver.getRawAxis(1) * .5;
    HOmniMotor.set(currSpeed);
    if(Math.abs(currSpeed) < .02){
      hopperMotor.set(0);
    }else{
      hopperMotor.set(0.25);
    }
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
