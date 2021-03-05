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
    pidController = m_pidController;

    pidController.setP(ShooterConstants.kP);
    pidController.setI(0);
    pidController.setD(ShooterConstants.kD);
    pidController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);

    ShuffleboardLayout layout = Shuffleboard.getTab(ControlConstants.SBTabDriverDisplay)
        .getLayout("Shooter", BuiltInLayouts.kList)
        .withPosition(ControlConstants.shooterColumn, 0)
        .withSize(2, 5);
    layout.addNumber("Velocity", motor.getEncoder()::getVelocity)
        .withWidget(BuiltInWidgets.kGraph)
        .withProperties(Map.of("Max", 6000))
        .withPosition(ControlConstants.shooterColumn, 1);
    layout.addBoolean("Ready To Shoot", this::getReadyToShoot)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withSize(2, 1)
        .withPosition(ControlConstants.shooterColumn, 2);
    layout.addNumber("Expected Velocity", this::getSetPoint).withWidget(BuiltInWidgets.kDial)
        .withPosition(ControlConstants.shooterColumn, 3)
        .withProperties(Map.of("Max", 6000));

    ShuffleboardLayout hoodLayout = Shuffleboard.getTab(ControlConstants.SBTabDriverDisplay)
        .getLayout("Hood", BuiltInLayouts.kList)
        .withPosition(ControlConstants.hoodColumn, 0)
        .withSize(2, 5);
    hoodLayout.addNumber("Pot Position", motor.getEncoder()::getVelocity)
        .withWidget(BuiltInWidgets.kGraph)
        .withProperties(Map.of("Max", 2000))
        .withPosition(ControlConstants.hoodColumn, 1);
    hoodLayout.addBoolean("Hood Ready", this::getHoodReadyToShoot)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withSize(2, 1)
        .withPosition(ControlConstants.hoodColumn, 2);
    hoodLayout.addNumber("Hood Pos", this::getSetPoint)
        .withWidget(BuiltInWidgets.kDial)
        .withPosition(ControlConstants.hoodColumn, 3)
        .withProperties(Map.of("Max", 6000));

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

  public void aimToDistance(double distance) {
    double rpm = 0.0307921 * Math.pow(distance, 2) + -1.24352 * distance + 1929.11;
    spinUpWheelRPM(rpm);
    aimHood(distance);
  }
  // data for new flywheel distance to hood and rpm
  // https://www.desmos.com/calculator/ykvrqcrgit
  public void spinUpWheelRPM(double setPoint) {
    this.flyWheelSetPoint = setPoint;
    pidController.setFF(ShooterConstants.kS / setPoint + ShooterConstants.kV);
    pidController.setReference(setPoint, ControlType.kVelocity);
  }

  public void checkWheelSpeed() {
    boolean readyToRPM;
    boolean readyToAngle;
    // rpm tolerance?
    // changed 75 to 400
    if (Math.abs(motor.getEncoder().getVelocity() - flyWheelSetPoint) < 75) {
      readyToRPM = true;
    } else {
      readyToRPM = false;
    }

    if (Math.abs(hoodPot.getAverageValue() - hoodSetPoint) < 25) {
      readyToAngle = true;
    } else {
      readyToAngle = false;
    }

    readyToShoot = readyToAngle && readyToRPM;
  }

  public void setWheelSpeed(double setPoint) {
    motor.set(-setPoint);
  }

  public boolean getReadyToShoot() {
    return readyToShoot;
  }

  public boolean getHoodReadyToShoot() {
    return Math.abs(hoodPot.getAverageValue() - hoodSetPoint) < 25;
  }

  public void end() {
    motor.set(0);
    aimHood(30);
  }

  public void moveHood(double pow) {
    if(hoodPot.getAverageValue() > 650 || hoodPot.getAverageValue() < 1900)
      this.hood.set(pow);
    else
      this.hood.set(0);
  }

  public void aimHood(double distance){
    hoodSetPoint = 0.010596 * Math.pow(distance, 2) + -6.96343 * distance + 1924.95;
    hoodSetPoint = Math.max(620, hoodSetPoint);
    hoodSetPoint = Math.min(1950, hoodSetPoint);
  }

  public void PIDHood(){
    double potValue = hoodPot.getAverageValue();
    double error = potValue - hoodSetPoint;
    hood.set(0.00075188 * error);
  }

  public int getHoodValue(){
    return hoodPot.getAverageValue();
  }

  @Override
  public void periodic() {
    PIDHood();
  }

  public double getSetPoint() {
    return flyWheelSetPoint;
  }

  public double getHoodSetPoint() {
    return hoodSetPoint;
  }

  public void setPoint(double setPoint) {
    this.flyWheelSetPoint = setPoint;
  }
}
