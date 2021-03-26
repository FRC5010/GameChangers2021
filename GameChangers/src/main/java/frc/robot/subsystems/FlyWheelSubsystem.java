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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ControlConstants;
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
  private double hoodSetPoint = 1600;

  public FlyWheelSubsystem(CANSparkMax m1, CANSparkMax hood, CANPIDController m_pidController,
      VisionSystem visionSystem) {
    // motor = m1;
    this.visionSystem = visionSystem;
    this.motor = m1;
    this.hood = hood;
    hoodPot = new AnalogInput(0);
    hoodSetPoint = hoodPot.getValue();
    pidController = m_pidController;

    pidController.setP(ShooterConstants.kP);
    pidController.setI(0);
    pidController.setD(ShooterConstants.kD);
    pidController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);

    ShuffleboardLayout layout = Shuffleboard.getTab(ControlConstants.SBTabDriverDisplay)
        .getLayout("Shooter", BuiltInLayouts.kList)
        .withPosition(ControlConstants.shooterColumn, 0)
        .withSize(2, 5);
    layout.addNumber("Actual Velocity", motor.getEncoder()::getVelocity)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("Max", 6000))
        .withPosition(ControlConstants.shooterColumn, 1);
    layout.addBoolean("Ready To Shoot", this::getReadyToShoot)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withSize(2, 1)
        .withPosition(ControlConstants.shooterColumn, 2);
    layout.addNumber("Expected Velocity", this::getSetPoint).withWidget(BuiltInWidgets.kDial)
        .withPosition(ControlConstants.shooterColumn, 3)
        .withProperties(Map.of("Max", 6000));
    layout.addNumber("Manual RPM", ShooterConstants::getBaseSpeed).withWidget(BuiltInWidgets.kDial)
        .withPosition(ControlConstants.shooterColumn, 3)
        .withProperties(Map.of("Max", 6000));

    ShuffleboardLayout hoodLayout = Shuffleboard.getTab(ControlConstants.SBTabDriverDisplay)
        .getLayout("Hood", BuiltInLayouts.kList)
        .withPosition(ControlConstants.hoodColumn, 0)
        .withSize(2, 5);
    hoodLayout.addNumber("Actual Position", this::getHoodValue)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("Max", ShooterConstants.hoodMaxDisplay * 2))
        .withPosition(ControlConstants.hoodColumn, 1);
    hoodLayout.addBoolean("Hood Ready", this::getHoodReadyToShoot)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withSize(2, 1)
        .withPosition(ControlConstants.hoodColumn, 2);
    hoodLayout.addNumber("Expected Position", this::getHoodSetPoint)
        .withWidget(BuiltInWidgets.kDial)
        .withPosition(ControlConstants.hoodColumn, 3)
        .withProperties(Map.of("Max", ShooterConstants.hoodMaxDisplay * 2));

    ShuffleboardLayout layoutDiag = Shuffleboard.getTab(ControlConstants.SBTabDiagnostics).getLayout("Shooter",
        BuiltInLayouts.kList);
    layoutDiag.addNumber("HoodPot", hoodPot::getAverageValue);
    layoutDiag.addNumber("Shooter Temp", motor::getMotorTemperature);
    layoutDiag.addNumber("Shooter Current", motor::getOutputCurrent);
  }

  public void spinUpWheel(double power) {
    motor.set(power);
    readyToShoot = false;
  }

  public void aimAtDistance(double distance) {
    distance = Double.valueOf(distance).intValue();
    double rpm = ShooterConstants.rpmC * distance + ShooterConstants.rpmD;
    spinUpWheelRPM(rpm);
    aimHood(distance);
  }

  public void aimHood(double distance){
    hoodSetPoint = ShooterConstants.hoodC * distance + ShooterConstants.hoodD;
    PIDHood();
  }

  // data for new flywheel distance to hood and rpm
  // https://www.desmos.com/calculator/e8sdi68bmn
  public void spinUpWheelRPM(double setPoint) {
    this.flyWheelSetPoint = setPoint;
    pidController.setFF(ShooterConstants.kS / setPoint + ShooterConstants.kV);
    pidController.setReference(setPoint, ControlType.kVelocity);
  }

  public void determineIfReadyToShoot() {
    readyToShoot = getFlyWheelReadyToShoot() && getHoodReadyToShoot();
  }

  public void setWheelSpeed(double setPoint) {
    motor.set(-setPoint);
  }

  public boolean getReadyToShoot() {
    return readyToShoot;
  }

  public boolean getFlyWheelReadyToShoot() {
    double rpmRange = 25;
    if (readyToShoot) {
      rpmRange = 150;
    }
    return (flyWheelSetPoint - motor.getEncoder().getVelocity()) < rpmRange;
  }

  public boolean getHoodReadyToShoot() {
    return Math.abs(hoodPot.getAverageValue() - hoodSetPoint) < ShooterConstants.hoodMove;
  }

  public void end() {
    motor.set(0);
    hood.set(0);
  }

  public void moveHood(double pow) {
    if(hoodPot.getAverageValue() > ShooterConstants.hoodMin || hoodPot.getAverageValue() < ShooterConstants.hoodMax)
      this.hood.set(pow);
    else
      this.hood.set(0);
  }

  public void PIDHood() {
    hoodSetPoint = Math.max(ShooterConstants.hoodMin, hoodSetPoint);
    hoodSetPoint = Math.min(ShooterConstants.hoodMax, hoodSetPoint);
    if (!readyToShoot) {
      double potValue = hoodPot.getAverageValue();
      double error = potValue - hoodSetPoint;
      hood.set(0.0015 * error);
    } else {
      hood.set(-0.01);
    }
  }

  public int getHoodValue(){
    return hoodPot.getAverageValue();
  }

  public double getHoodValueDisplay(){
    return (ShooterConstants.hoodMaxDisplay - hoodPot.getAverageValue());
  }

  @Override
  public void periodic() {
  }

  public double getSetPoint() {
    return flyWheelSetPoint;
  }

  public double getHoodSetPoint() {
    return hoodSetPoint;
  }

  public double getHoodSetPointDisplay() {
    return ShooterConstants.hoodMaxDisplay - hoodSetPoint;
  }

  public void setPoint(double setPoint) {
    this.flyWheelSetPoint = setPoint;
  }

  public void decHoodSetPoint() {
    hoodSetPoint -= ShooterConstants.hoodMove;
  }

  public void incHoodSetPoint() {
    hoodSetPoint += ShooterConstants.hoodMove;
  }
}
