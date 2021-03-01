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

import edu.wpi.first.wpilibj.AnalogInput;
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

  private VisionSystem visionSystem;

  public CANSparkMax motor;
  public CANSparkMax hood;
  private AnalogInput hoodPot;
  private CANPIDController pidController;
  private Boolean readyToShoot = false;
  private double flyWheelSetPoint = 0;

  public FlyWheelSubsystem(CANSparkMax m1, CANSparkMax hood, CANPIDController m_pidController,
      VisionSystem visionSystem) {
    // motor = m1;
    this.visionSystem = visionSystem;
    this.motor = m1;
    this.hood = hood;
    hoodPot = new AnalogInput(0);
    pidController = m_pidController;

    pidController.setP(ShooterConstants.kP);
    pidController.setI(0);
    pidController.setD(ShooterConstants.kD);
    pidController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);

    ShuffleboardLayout layout = Shuffleboard.getTab(Constants.SBTabDriverDisplay)
        .getLayout("Shooter", BuiltInLayouts.kList).withPosition(Constants.shooterColumn, 0).withSize(2, 5);
    layout.addNumber("Velocity", motor.getEncoder()::getVelocity).withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("Max", 6000)).withPosition(Constants.shooterColumn, 1);
    layout.addBoolean("Ready To Shoot", this::getReadyToShoot).withWidget(BuiltInWidgets.kBooleanBox).withSize(2, 1)
        .withPosition(Constants.shooterColumn, 1);
    layout.addNumber("Set Point", this::getSetPoint).withWidget(BuiltInWidgets.kDial)
        .withPosition(Constants.shooterColumn, 3).withProperties(Map.of("Max", 6000));
    layout.addNumber("Base Speed", ShooterConstants::getBaseSpeed).withSize(1, 1).withPosition(Constants.shooterColumn,
        4);
    layout.addNumber("Distance to RPM", ShooterConstants::getDistanceToRPM).withSize(1, 1)
        .withPosition(Constants.shooterColumn, 5);

    ShuffleboardLayout layoutDiag = Shuffleboard.getTab(Constants.SBTabDiagnostics).getLayout("Shooter",
        BuiltInLayouts.kList);
    layoutDiag.addNumber("HoodPot", hoodPot::getAverageValue);
    layoutDiag.addNumber("Shooter Temp", motor::getMotorTemperature);
    layoutDiag.addNumber("Shooter Current", motor::getOutputCurrent);
  }

  public void spinUpWheel(double power) {
    motor.set(power);
    readyToShoot = false;
  }

  // data for new flywheel distance to hood and rpm
  // https://www.desmos.com/calculator/ykvrqcrgit
  public void spinUpWheelRPM(double setPoint) {
    this.flyWheelSetPoint = setPoint;
    pidController.setFF(ShooterConstants.kS / setPoint + ShooterConstants.kV);
    pidController.setReference(setPoint, ControlType.kVelocity);
  }

  public void checkWheelSpeed() {
    // rpm tolerance?
    // changed 75 to 400
    if (Math.abs(motor.getEncoder().getVelocity() - flyWheelSetPoint) < 75) {
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
    if(hoodPot.getAverageValue() > 650 || hoodPot.getAverageValue() < 1900)
      this.hood.set(pow);
    else
      this.hood.set(0);
  }

  public int getHoodValue(){
    return hoodPot.getAverageValue();
  }

  @Override
  public void periodic() {
    double hoodSetPoint = 700;
    if (visionSystem.isValidTarget()) {
      double distance = visionSystem.getDistance();
      hoodSetPoint = 0.010596 * Math.pow(distance, 2) + -6.96343 * distance + 1924.95;
      hoodSetPoint = Math.max(620, hoodSetPoint);
      hoodSetPoint = Math.min(1950, hoodSetPoint);
    }
    double potValue = hoodPot.getAverageValue();
    double error = potValue - hoodSetPoint;
    //hood.set(0.00075188 * error);

  }

  public double getSetPoint() {
    return flyWheelSetPoint;
  }

  public void setPoint(double setPoint) {
    this.flyWheelSetPoint = setPoint;
  }
}
