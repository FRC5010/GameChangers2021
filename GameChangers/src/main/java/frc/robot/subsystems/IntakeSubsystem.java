// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ControlConstants;
import frc.robot.mechanisms.IntakeConstants;
import frc.robot.mechanisms.IntakeMech;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private CANSparkMax m9;
  private CANSparkMax m12;
  private CANPIDController pid;
  private CANEncoder intakeEncoder;

  public IntakeSubsystem(CANSparkMax m9, CANSparkMax m12) {
    this.m9 = m9;
    this.m12 = m12;
    pid = m9.getPIDController();
    intakeEncoder = m9.getEncoder();

    pid.setP(IntakeConstants.intakeP);
    pid.setD(IntakeConstants.intakeD);
    pid.setFF(IntakeConstants.intakeFF);
    pid.setOutputRange(IntakeConstants.kMinOutput, IntakeConstants.kMaxOutput);

    int smartMotionSlot = 0;
    pid.setSmartMotionMaxVelocity(IntakeConstants.maxVel, smartMotionSlot);
    pid.setSmartMotionMinOutputVelocity(IntakeConstants.minVel, smartMotionSlot);
    pid.setSmartMotionMaxAccel(IntakeConstants.maxAccel, smartMotionSlot);
    pid.setSmartMotionAllowedClosedLoopError(IntakeConstants.allowedErr, smartMotionSlot);

    ShuffleboardLayout layout = Shuffleboard.getTab("Intake").getLayout("Intaker", BuiltInLayouts.kList)
        .withPosition(ControlConstants.shooterColumn, 1).withSize(2, 5);
    layout.addNumber("Velocity", m12.getEncoder()::getVelocity).withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("Max", 15000, "Min", -15000)).withPosition(ControlConstants.shooterColumn, 3);
    layout.addNumber("Current", m12::getOutputCurrent).withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("Max", 120)).withPosition(ControlConstants.shooterColumn, 5);
    layout.addNumber("Encoder Value", intakeEncoder::getPosition).withPosition(ControlConstants.shooterColumn, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntakePosition() {

  }

  public double getIntakePosition(){
    return intakeEncoder.getPosition();
  }

  public void spin(double power) {
    m12.set(power);
  }

  public void moveIntake(double power) {
    double encoderVal = intakeEncoder.getPosition();
    // double error = Math.min(Math.abs(encoderVal-IntakeMech.intakeFastMax),
    // Math.abs(encoderVal-IntakeMech.intakeFastMin));
    double error;
    double finPow;
    // encoderVal <= IntakeMech.intakeFastMax && encoderVal >=
    // IntakeMech.intakeFastMin

    if (power > 0) {
      if (encoderVal <= IntakeMech.intakeMax) {
        error = Math.abs(encoderVal - IntakeMech.intakeMax);
        finPow = encoderVal <= IntakeMech.intakeFastMax ? power : power * (error / 100);
        m9.set(finPow);
      } else {
        m9.set(0);
      }
    } else if (power < 0) {
      if (encoderVal >= IntakeMech.intakeMin) {
        error = Math.abs(encoderVal - IntakeMech.intakeMin);
        finPow = encoderVal >= IntakeMech.intakeFastMin ? power : power * (error / 100);
        m9.set(finPow);
      } else {
        m9.set(0);
      }
    } else {
      m9.set(0);
    }

  }

  public void stop() {
    m12.set(0);
  }
}
