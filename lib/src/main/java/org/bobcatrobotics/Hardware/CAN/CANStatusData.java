package org.bobcatrobotics.Hardware.CAN;

public class CANStatusData {
    public String name;
    public double utilization;
    public int busOffCount;
    public int txFullCount;
    public int receiveErrorCount;
    public int transmitErrorCount;
    public boolean isOk;
}