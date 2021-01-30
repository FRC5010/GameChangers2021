/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FlyWheelSubsystem extends SubsystemBase {
  /**
   * Creates a new FlyWheelSubsystem.
   */
  public CANSparkMax motor;
  public CANSparkMax hood;
  private CANPIDController pidController;

  public FlyWheelSubsystem(CANSparkMax m1, CANSparkMax hood) {
    motor = m1;
    this.hood = hood;
    pidController = motor.getPIDController();
    ShuffleboardLayout layout = Shuffleboard.getTab(Constants.SBTabDriverDisplay)
      .getLayout("Shooter", BuiltInLayouts.kList).withPosition(Constants.shooterColumn, 1).withSize(2,5);
    layout.addNumber("Velocity", motor.getEncoder()::getVelocity).withWidget(BuiltInWidgets.kDial)
      .withProperties(Map.of("Max", 6000)).withPosition(Constants.shooterColumn, 1);
    layout.addNumber("Current", motor::getOutputCurrent).withWidget(BuiltInWidgets.kDial)
      .withProperties(Map.of("Max", 6000)).withPosition(Constants.shooterColumn, 1);
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
  }
}
