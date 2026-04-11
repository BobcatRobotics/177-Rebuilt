// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.RobotState;
import frc.robot.generated.TunerConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Queue;

import org.bobcatrobotics.Util.CANDeviceDetails;
import org.bobcatrobotics.Util.CANDeviceDetails.Hardware;
import org.bobcatrobotics.Util.CANDeviceDetails.Manufacturer;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon =
      new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.kCANBus);
  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

  public GyroIOPigeon2() {
    if (TunerConstants.DrivetrainConstants.Pigeon2Configs != null) {
      pigeon.getConfigurator().apply(TunerConstants.DrivetrainConstants.Pigeon2Configs);
    } else {
      pigeon.getConfigurator().apply(new Pigeon2Configuration());
    }

    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
    yawVelocity.setUpdateFrequency(50.0);
    pigeon.optimizeBusUtilization();
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(yaw.clone());

    updateCanDetails("CANivore",TunerConstants.DrivetrainConstants.Pigeon2Id,Hardware.Pigeon2,Manufacturer.Ctre,"Drive",pigeon.isConnected());
    }

    public void updateCanDetails(String bus, int id, Hardware hardware, Manufacturer manufacturer , String subsystemname, boolean status) {
      CANDeviceDetails carwashDeviceDetails = new CANDeviceDetails(id,bus,hardware,manufacturer,subsystemname,status);
      List<CANDeviceDetails> rioDevices = RobotState.getInstance().devices.get(bus);
      rioDevices.add(carwashDeviceDetails);
      RobotState.getInstance().devices.replace(bus, rioDevices);
      RobotState.getInstance().subsytemDriveDevices.add(carwashDeviceDetails);
    }


  public void updateCanDetails(String bus, int id, Hardware hardware, boolean status) {
    List<CANDeviceDetails> fulldevices = RobotState.getInstance().devices.get(bus);

    Optional<CANDeviceDetails> opt = fulldevices.stream()
        .filter(obj -> obj.id() == id && obj.hardware().equals(hardware))
        .findFirst();

    if (opt.isEmpty()) {
        return; // or handle error/log
    }

    CANDeviceDetails old = opt.get();

    CANDeviceDetails updated = new CANDeviceDetails(
        old.id(),
        old.bus(),
        old.hardware(),
        old.manufacturer(),
        old.subsystemName(),
        status
    );

    fulldevices.replaceAll(item ->
        (item.id() == id && item.hardware().equals(hardware)) ? updated : item
    );

    RobotState.getInstance().devices.put(bus, fulldevices);
}

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();

      updateCanDetails("CANivore",pigeon.getDeviceID(),Hardware.Pigeon2,pigeon.isConnected());
  }
}
