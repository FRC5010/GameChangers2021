/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

public class VisionOpenSight extends VisionSystem {
  static final double screenSizeX = 160;
  static final double screenSizeY = 120;
  static final double camFovX = 61;
  static final double camFovY = 34.3;

  /**
   * Creates a new OpenSightVision.
   */
  public VisionOpenSight(String name, int colIndex) {
    super(name, colIndex);
  }

  public VisionOpenSight(String name, double camHeight, double camAngle, double targetHeight, int colIndex) {
    super(name, camHeight, camAngle, targetHeight, colIndex);
  }

  @Override
  public void periodic() {
    if (updateValues) {
      updateViaNetworkTable("/OpenSight/" + name);
    }
  }

  public void updateViaNetworkTable(String path) {
    // essential variables from NetworkTables
    boolean valid = table.getTable(path).getEntry("valid").getBoolean(false);
    double centerX = table.getTable(path).getEntry("center-x").getDouble(0);
    double centerY = table.getTable(path).getEntry("center-y").getDouble(0);
    double area = 0;
    // calculating angle
    double angleY = camFovY * (screenSizeY / 2 - (centerY * (screenSizeY / 2))) / screenSizeY;
    double angleX = camFovX / 2 * centerX / 0.9;

    // calculating distance
    double distance = (targetHeight - camHeight) / Math.tan(Math.toRadians(angleY + camAngle));

    rawValues = new VisionValues(valid, centerX, centerY, angleX, angleY, distance);
  }

  // TODO: FIX ME - WRONG DEFAULT ANGLE
  public void calibarateCamAngle(double angleY) {
    camAngle = 28.7 - angleY;
  }

  public void setLight(boolean on) {}
  public void flashLight() {}
  public boolean isLightOn() { return false; }
}
