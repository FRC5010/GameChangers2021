/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.mechanisms.ShooterConstants;
import frc.robot.subsystems.FlyWheelSubsystem;
import frc.robot.subsystems.HopperOmniSubsystem;
import frc.robot.subsystems.VisionSystem;

public class ShootBall extends CommandBase {
  /**
   * Creates a new ShootBall.
   */
  public FlyWheelSubsystem flyWheelSubsystem;
  private HopperOmniSubsystem hopperOmniSubsystem;
  private VisionSystem visionSystem;
  public ShootBall(FlyWheelSubsystem flyWheelSubsystem, HopperOmniSubsystem hopperOmniSubsystem, VisionSystem visionSystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hopperOmniSubsystem = hopperOmniSubsystem;
    this.flyWheelSubsystem = flyWheelSubsystem;
    this.visionSystem = visionSystem;
    addRequirements(this.flyWheelSubsystem, this.hopperOmniSubsystem, this.visionSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = 60;

    if(visionSystem.isValidTarget())
      distance = visionSystem.getDistance();
    
    //rpm -= 200;
    //test since flywheel never fully reaches setpoint
    flyWheelSubsystem.aimToDistance(distance);

    flyWheelSubsystem.checkWheelSpeed();
    if(flyWheelSubsystem.getReadyToShoot()){
      hopperOmniSubsystem.SetOmniSpeed(-.9);
      hopperOmniSubsystem.SetHopperSpeed(.25);
    }else{
      hopperOmniSubsystem.SetHopperSpeed(0);
      hopperOmniSubsystem.SetOmniSpeed(0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flyWheelSubsystem.end();
    flyWheelSubsystem.checkWheelSpeed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
