/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ControlConstants;

public abstract class VisionSystem extends SubsystemBase {

  protected String name;
  protected double camHeight, camAngle, targetHeight;
  protected NetworkTableInstance table;
  protected VisionValues rawValues, smoothedValues;
  protected ShuffleboardLayout visionLayout;
  protected boolean updateValues = false;
  // variables needed to process new variables, plus the new variables
  // angles

  public VisionSystem(String name, int colIndex) {
    table = NetworkTableInstance.getDefault();
    this.name = name;
    rawValues = new VisionValues();
    ShuffleboardTab driverTab = Shuffleboard.getTab(ControlConstants.SBTabDriverDisplay);
    visionLayout = driverTab.getLayout(name + " Vision", BuiltInLayouts.kGrid).withPosition(colIndex, 0).withSize(3, 5);
  }

  public VisionSystem(String name, double camHeight, double camAngle, double targetHeight, int colIndex) {
    this.name = name;
    rawValues = new VisionValues();
    this.camHeight = camHeight;
    this.camAngle = camAngle;
    this.targetHeight = targetHeight;
    updateValues = true;
    ShuffleboardTab driverTab = Shuffleboard.getTab(ControlConstants.SBTabDriverDisplay);
    visionLayout = driverTab.getLayout(name + " Vision", BuiltInLayouts.kGrid).withPosition(colIndex, 0).withSize(3, 5);
    table = NetworkTableInstance.getDefault();
    // HttpCamera camera = new HttpCamera(path + " Cam",
    // "http://opensight.local:1181/hooks/opsi.videoio/" + path + "cam.mjpeg",
    // HttpCameraKind.kMJPGStreamer);
    // visionLayout.add(camera).withWidget(BuiltInWidgets.kCameraStream).withSize(3,
    // 2);

    visionLayout.addNumber(name + " Distance", this::getDistance).withSize(1, 1);
    visionLayout.addNumber(name + " Cam Angle", this::getCamAngle).withSize(1, 1);
    visionLayout.addNumber(name + " X Angle", this::getAngleX).withSize(1, 1);
    visionLayout.addNumber(name + " Y Angle", this::getAngleY).withSize(1, 1);

    //visionLayout.addBoolean(name + " Driver Mode", RobotContainer::getDriverMode).withSize(1, 1);
  }

  public VisionValues getRawValues() {
    return rawValues;
  }

  public abstract void updateViaNetworkTable(String path);

  public abstract void calibarateCamAngle(double angleY);

  public abstract void setLight(boolean on);
  public abstract boolean isLightOn();
  public abstract void flashLight();

  public void setCamAngle(double a) {
    camAngle = a;
  }

  public double getCamAngle() {
    return camAngle;
  }

  public double getDistance() {
    return rawValues.getDistance();
  }

  public double getAngleX() {
    return rawValues.getAngleX();
  }

  public double getAngleY() {
    return rawValues.getAngleY();
  }

  public boolean isValidTarget() {
    return rawValues.getValid();
  }
}
