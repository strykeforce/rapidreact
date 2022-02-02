package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

import org.jetbrains.annotations.NotNull;
import org.strykeforce.deadeye.Deadeye;
import org.strykeforce.deadeye.TargetDataListener;
import org.strykeforce.deadeye.TargetListTargetData;

public class DeadeyeC0 implements TargetDataListener<TargetListTargetData> {
  private final Deadeye<TargetListTargetData> deadeye;
  public TargetListTargetData lastData;
  public final double minContourAreaSize;
  public final int frameCenter;

  public DeadeyeC0() {
    this(NetworkTableInstance.getDefault());
  }

  public DeadeyeC0(NetworkTableInstance nti) {
    deadeye = new Deadeye<>("C0", TargetListTargetData.class, nti);
    deadeye.setTargetDataListener(this);
    frameCenter = deadeye.getCapture().width / 2;
    minContourAreaSize = Constants.VisionConstants.kMinContourAreaSize;
  }

  public void setEnabled(boolean enabled) {
    deadeye.setEnabled(enabled);
  }

  @Override
  public void onTargetData(TargetListTargetData data) {
    lastData = data;
  }

  public TargetListTargetData getTargetListData() {
    return lastData;
  }

  public boolean isValid() {
    if (lastData != null) {
      return lastData.valid;
    }
    return false;
  }

  public @NotNull String getId() {
    return lastData.id;
  }

  public int getSerial() {
    return lastData.serial;
  }

  public void setLightsEnabled(boolean enabled) {
    deadeye.setLightEnabled(enabled);
  }
}
