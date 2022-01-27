package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import java.util.ArrayList;
import org.strykeforce.deadeye.TargetListTargetData;
import org.strykeforce.deadeye.TargetListTargetData.Target;

public class VisionSubsystem implements Subsystem {
  public DeadeyeC0 ShooterCamera;
  private NetworkTableInstance deadeyeNetworkTableInstance;

  public VisionSubsystem() {
    deadeyeNetworkTableInstance = NetworkTableInstance.create();
    deadeyeNetworkTableInstance.startClient("192.168.3.3", 1736); // TEST Deadeye
    // deadeyeNetworkTableInstance.startClient("192.168.3.3", 1735); //real Deadeye
    ShooterCamera = new DeadeyeC0(deadeyeNetworkTableInstance);
  }

  public boolean isTargetValid() {
    return ShooterCamera.getValid();
  }

  public double getHorizAngleAdjustment() {
    return Constants.VisionConstants.kHorizAngleCorrection;
  }

  public boolean isStable() {
    return true;
  }

  public ArrayList<TargetListTargetData.Target> getSortedTargets() {
    ArrayList<TargetListTargetData.Target> sortedlist =
        new ArrayList<TargetListTargetData.Target>();
    int i;
    for (Target target : ShooterCamera.getTargetListData()) {
      i = 0;
      for (Target sortedTarget : sortedlist) {
        if (target.center.x > sortedTarget.center.x) {
          i += 1;
        }
      }
      sortedlist.add(i, target);
    }
    System.out.print(
        "SortedList: 1:"
            + sortedlist.get(0).center.x
            + " 2: "
            + sortedlist.get(1).center.x
            + " 3: "
            + sortedlist.get(2).center.x /*+ " 4: " + sortedlist.get(3).center.x*/);
    return sortedlist;
  }

  public double getAzmithError() {
    ArrayList<TargetListTargetData.Target> listData = getSortedTargets();
    int minX =
        listData.get(0)
            .topLeft
            .x; // need to sort listdata(left to right) and use that data instead, GET ISAAC's SORT
    // CODE
    int maxX = listData.get(0).bottomRight.x;
    double targetcenter;
    for (int i = 0; i < listData.size(); i++) {
      if (listData.get(i).topLeft.x < minX) {
        minX = listData.get(i).topLeft.x;
      }
      if (listData.get(i).bottomRight.x > maxX) {
        maxX = listData.get(i).bottomRight.x;
      }
    }
    targetcenter = (maxX + minX) / 2;
    return targetcenter;
  }
}
