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
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.mechanisms.ShooterConstants;

public class FlyWheelSubsystem extends SubsystemBase {
  /**
   * Creates a new FlyWheelSubsystem.
   */
  public CANSparkMax motor;
  public CANSparkMax hood;
  private CANPIDController pidController;
  private Boolean readyToShoot = false;
  private double setPoint = 0;

  public FlyWheelSubsystem(CANSparkMax m1, CANSparkMax hood, CANPIDController m_pidController) {
    // motor = m1;
    this.motor = m1;
    this.hood = hood;
    pidController = m_pidController;

    pidController.setP(ShooterConstants.kP);
    pidController.setI(0);
    pidController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);

    ShuffleboardLayout layout = Shuffleboard.getTab(Constants.SBTabDriverDisplay)
        .getLayout("Shooter", BuiltInLayouts.kList).withPosition(Constants.shooterColumn, 0).withSize(2, 5);
    layout.addNumber("Velocity", motor.getEncoder()::getVelocity).withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("Max", 6000)).withPosition(Constants.shooterColumn, 1);
    layout.addBoolean("Ready To Shoot", this::getReadyToShoot).withWidget(BuiltInWidgets.kBooleanBox).withSize(2, 1)
        .withPosition(Constants.shooterColumn, 1);
    layout.addNumber("Set Point", this::getSetPoint).withWidget(BuiltInWidgets.kDial)
        .withPosition(Constants.shooterColumn, 3).withProperties(Map.of("Max", 6000));
    layout.addNumber("Base Speed", ShooterConstants::getBaseSpeed).withSize(1, 1).withPosition(Constants.shooterColumn, 4);
    layout.addNumber("Distance to RPM", ShooterConstants::getDistanceToRPM).withSize(1, 1)
        .withPosition(Constants.shooterColumn, 5);

    ShuffleboardLayout layoutDiag = Shuffleboard.getTab(Constants.SBTabDiagnostics).getLayout("Shooter",
        BuiltInLayouts.kList);
    layoutDiag.addNumber("Shooter Temp", motor::getMotorTemperature);
    layoutDiag.addNumber("Shooter Current", motor::getOutputCurrent);
  }

  public void spinUpWheel(double power) {
    motor.set(power);
    readyToShoot = false;
  }
  //data for new flywheel distance to hood and rpm https://www.desmos.com/calculator/7lo2jt5y3t
  public void spinUpWheelRPM(double setPoint) {
    this.setPoint = setPoint;
    pidController.setFF(ShooterConstants.kS / setPoint + ShooterConstants.kV);
    pidController.setReference(setPoint, ControlType.kVelocity);
  }

  public void checkWheelSpeed() {
    // rpm tolerance?
    // changed 75 to 400
    if (Math.abs(motor.getEncoder().getVelocity() - setPoint) < 75) {
      readyToShoot = true;
    } else {
      readyToShoot = false;
    }
  }

  public void setWheelSpeed(double setPoint) {
    motor.set(-setPoint);
  }

  public boolean getReadyToShoot() {
    return readyToShoot;
  }

  public void end() {
    motor.set(0);
  }

  public void moveHood(double pow) {
    this.hood.set(pow);
  }

  @Override
  public void periodic() {

  }

  public double getSetPoint() {
    return setPoint;
  }

  public void setPoint(double setPoint) {
    this.setPoint = setPoint;
  }
}
