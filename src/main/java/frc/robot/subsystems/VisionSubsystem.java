package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import java.util.ArrayList;
import org.strykeforce.deadeye.Rect;

public class VisionSubsystem implements Subsystem {
  public DeadeyeC0 shooterCamera;
  // private NetworkTableInstance deadeyeNetworkTableInstance;

  public VisionSubsystem() {
    // deadeyeNetworkTableInstance = NetworkTableInstance.create();
    // deadeyeNetworkTableInstance.startClient("192.168.3.3", 1736); // TEST Deadeye
    // deadeyeNetworkTableInstance.startClient("192.168.3.3", 1735); // real Deadeye
    shooterCamera = new DeadeyeC0(); // deadeyeNetworkTableInstance);
  }

  public boolean isTargetValid() {
    return shooterCamera.getValid();
  }

  public double getHorizAngleAdjustment() {
    return Constants.VisionConstants.kHorizAngleCorrection;
  }

  public boolean isStable() {
    return true;
  }

  public ArrayList<Rect> getSortedRects() {
    ArrayList<Rect> sortedlist = new ArrayList<Rect>();
    int i;
    for (Rect Rect : shooterCamera.getTargetListData()) {
      i = 0;
      for (Rect sortedRect : sortedlist) {
        if (Rect.center().x > sortedRect.center().x) {
          i += 1;
        }
      }
      sortedlist.add(i, Rect);
    }
    System.out.print(
        "SortedList: 1:"
            + sortedlist.get(0).center().x
            + " 2: "
            + sortedlist.get(1).center().x
            + " 3: "
            + sortedlist.get(2).center().x /*+ " 4: " + sortedlist.get(3).center().x*/);
    return sortedlist;
  }

  public double getPixelError() {
    ArrayList<Rect> listData = getSortedRects();
    int minX = listData.get(0).topLeft.x;
    int maxX = listData.get(0).bottomRight.x;
    double center;
    for (int i = 0; i < listData.size(); i++) {
      if (listData.get(i).topLeft.x < minX) {
        minX = listData.get(i).topLeft.x;
      }
      if (listData.get(i).bottomRight.x > maxX) {
        maxX = listData.get(i).bottomRight.x;
      }
    }
    center = (maxX + minX) / 2;
    System.out.println("getPixelError: " + (center - shooterCamera.frameCenter));
    return center - shooterCamera.frameCenter;
  }

  public double getOffsetAngle() {
    if (shooterCamera.getValid()) {
      return Constants.VisionConstants.HORIZ_FOV * getPixelError() / shooterCamera.frameCenter;
    }
    return 2767;
  }
}
