package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import java.util.ArrayList;
import org.jetbrains.annotations.NotNull;
import org.strykeforce.deadeye.Deadeye;
import org.strykeforce.deadeye.Rect;
import org.strykeforce.deadeye.TargetDataListener;
import org.strykeforce.deadeye.TargetListTargetData;

public class DeadeyeC0 implements TargetDataListener<TargetListTargetData> {
  private final Deadeye<TargetListTargetData> deadeye;
  public TargetListTargetData lastData;
  public final double minContourAreaSize;
  public final int frameCenter;

  public DeadeyeC0() {
    deadeye = new Deadeye<>("C0", TargetListTargetData.class);
    deadeye.setTargetDataListener(this);
    frameCenter = deadeye.getCapture().width / 2;
    minContourAreaSize = Constants.VisionConstants.minContourAreaSize;
  }

  public DeadeyeC0(NetworkTableInstance nti) {
    deadeye = new Deadeye<>("C0", TargetListTargetData.class, nti);
    deadeye.setTargetDataListener(this);
    frameCenter = deadeye.getCapture().width / 2;
    minContourAreaSize = Constants.VisionConstants.minContourAreaSize;
  }

  public void setEnabled(boolean enabled) {
    deadeye.setEnabled(enabled);
  }

  @Override
  public void onTargetData(TargetListTargetData data) {
    lastData = data;
  }

  public ArrayList<Rect> getTargetListData() {
    ArrayList<Rect> listdata = new ArrayList<>();
    for (int i = 0; i < lastData.targets.size(); i++) {
      if (lastData.targets.get(i).contourArea > minContourAreaSize) {
        listdata.add(lastData.targets.get(i));
      }
    }
    if (lastData == null) {
      System.out.println("lastData is null");
    }
    return listdata;
  }

  public boolean getValid() {
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
