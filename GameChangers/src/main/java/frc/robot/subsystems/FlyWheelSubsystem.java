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
import frc.robot.commands.ShooterConstants;

public class FlyWheelSubsystem extends SubsystemBase {
  /**
   * Creates a new FlyWheelSubsystem.
   */
  public CANSparkMax motor;
  public CANSparkMax hood;
  private CANPIDController pidController;
  private Boolean readyToShoot = false;

  public FlyWheelSubsystem(CANSparkMax m1, CANSparkMax hood, CANPIDController m_pidController) {
    //motor = m1;
    this.motor = m1;
    this.hood = hood;
    pidController = m_pidController;

    pidController.setP(ShooterConstants.kP);
    pidController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);

    ShuffleboardLayout layout = Shuffleboard.getTab(Constants.SBTabDriverDisplay)
      .getLayout("Shooter", BuiltInLayouts.kList).withPosition(Constants.shooterColumn, 1).withSize(2,5);
    layout.addNumber("Velocity", motor.getEncoder()::getVelocity).withWidget(BuiltInWidgets.kDial)
      .withProperties(Map.of("Max", 6000)).withPosition(Constants.shooterColumn, 1);
    layout.addNumber("Current", motor::getOutputCurrent).withWidget(BuiltInWidgets.kDial)
      .withProperties(Map.of("Max", 6000)).withPosition(Constants.shooterColumn, 1);
    layout.addBoolean("Ready To Shoot", this::getReadyToShoot).withWidget(BuiltInWidgets.kBooleanBox).withSize(2, 1)
      .withPosition(Constants.shooterColumn, 1);
  }

  public void spinUpWheelRPM(double setPoint){
    pidController.setFF(ShooterConstants.kS / setPoint + ShooterConstants.kV);
    pidController.setReference(setPoint, ControlType.kVelocity);

    //rpm tolerance?
    // changed 75 to 400
    if(Math.abs(motor.getEncoder().getVelocity() - setPoint) < 600){
      readyToShoot = true;
    } else {
      readyToShoot = false;
    }
  }
  
  public void setWheelSpeed(double setPoint){
    motor.set(-setPoint);
  }
  public boolean getReadyToShoot(){
    return readyToShoot;
  }

  public void end(){
    motor.set(0);
  }

  public void moveHood(double pow){
    this.hood.set(pow);
  }

  @Override
  public void periodic() {

  }
}
