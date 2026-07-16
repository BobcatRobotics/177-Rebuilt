package org.bobcatrobotics.Hardware.Configurators.Sensors.Gyros;

public class GyroDeviceJson {
  public GyroDeviceJson() {
  }

  public GyroDeviceJson(int id, String canbus) {
    this.id = id;
    this.canbus = canbus;
  }

  /** The device type, e.g. pigeon/pigeon2/sparkmax/talonfx/navx */
  public String type = "";
  /** The CAN ID or pin ID of the device. */
  public int id = 0;
  /** The CAN bus name which the device resides on if using CAN. */
  public String canbus = "";
  /** sets the inversion state of the gyro. */
  public boolean inverted = false;
}