// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
import frc.robot.commands.Driving;

public class DriveTrainMain extends SubsystemBase {
  /**
   * Creates a new DriveTrainMain.
   */
  private SpeedController leftMaster;
  private SpeedController rightMaster;

  public DriveTrainMain(SpeedController left, SpeedController right, Joystick driver, Pose pose) {
    leftMaster = left;
    rightMaster = right;

  //  pdp = new PowerDistributionPanel();
    setDefaultCommand(new Driving(this, driver));
   // setDefaultCommand(new FlickStick(this, driver, pose));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMaster.setVoltage(leftVolts);
    rightMaster.setVoltage(rightVolts);
  }

  public void arcadeDrive(double fPow, double tPow) {
    tPow *= 0.7;
    fPow *= 0.8;
    //0.7 is set currently for Michael's practice runs
    leftMaster.set(fPow + tPow);
    rightMaster.set(fPow - tPow);
  }

  public double scaleInputs(double input) {
    if (input > -.1 && input < .1) {
      return 0.0;
    }
    if (input > 1) {
      return 1;
    }
    if (input < -1) {
      return -1;
    }
    
    return Math.pow(input, 3);

  }

  public void setMaxOutput(double maxOutput) {
    leftMaster.set(maxOutput);
    rightMaster.set(maxOutput);
  }
}
