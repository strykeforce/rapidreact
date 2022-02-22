package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.kHorizonFov;

import com.squareup.moshi.JsonReader;
import com.squareup.moshi.JsonWriter;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.VisionConstants;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import okio.Buffer;
import okio.BufferedSource;
import org.jetbrains.annotations.NotNull;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.strykeforce.deadeye.DeadeyeJsonAdapter;
import org.strykeforce.deadeye.Point;
import org.strykeforce.deadeye.Rect;
import org.strykeforce.deadeye.TargetListTargetData;

public class HubTargetData extends TargetListTargetData {

  static int kFrameCenter = Integer.MAX_VALUE;
  private final double errorPixels;
  private final double range;
  private final Logger logger = LoggerFactory.getLogger(this.getClass());

  public HubTargetData() {
    super();
    errorPixels = 0.0;
    range = 0.0;
  }

  public HubTargetData(
      @NotNull String id,
      int serial,
      boolean valid,
      double errorPixels,
      double range,
      @NotNull List<Rect> targets) {
    super(id, serial, valid, targets);
    this.errorPixels = errorPixels;
    this.range = range;
  }

  /**
   * Check if this contains valid target data. Valid data is defined by the {@code valid} flag being
   * set {@code true} by the Deadeye camera and that there is one or more targets detected.
   *
   * @return true if valid
   */
  public boolean isValid() {
    return valid && !targets.isEmpty();
  }

  /**
   * Return the distance from the center of the Hub target group to the center of the camera frame.
   * You should check {@link #isValid()} before calling this method.
   *
   * @return the number of pixels, positive if ..., negative if ...
   * @throws IndexOutOfBoundsException if the list of targets is empty
   */
  public double getErrorPixels() {
    // FIXME use inside edges
    int minX = targets.get(0).topLeft.x;
    int maxX = targets.get(targets.size() - 1).bottomRight.x;
    return (maxX + minX) / 2.0 - kFrameCenter;
  }

  /**
   * Return the angle from the center of the Hub target group to the center of the camera frame. You
   * should check {@link #isValid()} before calling this method.
   *
   * @return the angle in Radians
   * @throws IndexOutOfBoundsException if the list of targets is empty
   */
  public double getErrorRadians() {
    return -kHorizonFov * getErrorPixels() / (kFrameCenter * 2);
  }

  /**
   * Return the angle from the center of the Hub target group to the center of the camera frame. You
   * should check {@link #isValid()} before calling this method.
   *
   * @return the angle as a {@code Rotation2d}
   * @throws IndexOutOfBoundsException if the list of targets is empty
   */
  public Rotation2d getErrorRotation2d() {
    return new Rotation2d(getErrorRadians());
  }

  public double getInterpolateT() {
    return (kFrameCenter - Math.abs(getErrorPixels())) / kFrameCenter;
  }

  /**
   * Uses the width of the highest target to find the distance
   *
   * @return distance to target in inches, 2767 if not valid
   */
  public double getDistance1() {
    if (!isValid()) {
      return 2767;
    }

    List<Rect> targetData = targetsOrderedByTopLeftY();
    double width = targetData.get(0).width();

    double enclosedAngle = Math.toDegrees(kHorizonFov) * width / (kFrameCenter * 2);
    return VisionConstants.kTargetWidthIn / 2 / Math.tan(Math.toRadians(enclosedAngle / 2));
  }

  /**
   * Uses the distance between the two highest targets to find the distance
   *
   * @return distance to target in inches, 2767 if not valid
   */
  public double getDistance2() {
    if (!isValid() && (targets.size() > 1)) {
      return 2767;
    }

    List<Rect> targetData = targetsOrderedByTopLeftY();
    Rect target1 = targetData.get(0);
    Rect target2 = targetData.get(1);

    double targetDistance;
    if (target1.topLeft.x > target2.topLeft.x) {
      targetDistance = target1.topLeft.x - target2.bottomRight.x;
    } else {
      targetDistance = target2.topLeft.x - target1.bottomRight.x;
    }

    double enclosedAngle = Math.toDegrees(kHorizonFov) * targetDistance / (kFrameCenter * 2);
    return 5.5 / 2 / Math.tan(Math.toRadians(enclosedAngle / 2));
    // 5.5 distance between targets in
  }

  /**
   * Uses the height of the highest target to find the distance
   *
   * @return distance to target in inches, 2767 if not valid
   */
  public double getDistance3() {
    if (!isValid()) {
      return 2767;
    }

    List<Rect> targetData = targetsOrderedByTopLeftY();
    double height = targetData.get(0).height();

    double enclosedAngle = Math.toDegrees(kHorizonFov) * height / (kFrameCenter * 2);
    return 2 / 2 / Math.tan(Math.toRadians(enclosedAngle / 2));
    // 1st 2 is target height
  }

  public double testGetTargetsPixelWidth() {
    double pixelWidth;

    if (targets.size() % 2 == 1) {
      Rect leftTarget = targets.get((targets.size() - 1) / 2 - 1);
      Rect rightTarget = targets.get((targets.size() - 1) / 2 + 1);

      pixelWidth = rightTarget.bottomRight.x - leftTarget.topLeft.x;
    } else {
      Rect leftTarget = targets.get(targets.size() / 2 - 2);
      Rect rightTarget = targets.get(targets.size() / 2 + 1);

      pixelWidth = rightTarget.topLeft.x - leftTarget.bottomRight.x;
    }

    return pixelWidth;
  }

  public double testGetDistance() {
    double pixelWidth;

    if (targets.size() % 2 == 1) {
      Rect leftTarget = targets.get((targets.size() - 1) / 2 - 1);
      Rect rightTarget = targets.get((targets.size() - 1) / 2 + 1);

      pixelWidth = rightTarget.bottomRight.x - leftTarget.topLeft.x;

      double enclosedAngle = Math.toDegrees(kHorizonFov) * pixelWidth / (kFrameCenter * 2);
      return 25.25 / 2 / Math.tan(Math.toRadians(enclosedAngle / 2));
    } else {
      Rect leftTarget = targets.get(targets.size() / 2 - 2);
      Rect rightTarget = targets.get(targets.size() / 2 + 1);

      pixelWidth = rightTarget.topLeft.x - leftTarget.bottomRight.x;

      double enclosedAngle = Math.toDegrees(kHorizonFov) * pixelWidth / (kFrameCenter * 2);
      return 25.625 / 2 / Math.tan(Math.toRadians(enclosedAngle / 2));
    }
  }

  public double getGroundDistance() {
    if (!isValid()) {
      return 2767;
    }

    return Math.sqrt(
            Math.pow(testGetTargetsPixelWidth(), 2)
                - Math.pow(VisionConstants.kTapeHeightIn - VisionConstants.kCameraHeight, 2))
        + VisionConstants.kUpperHubRadiusIn;
  }

  @SuppressWarnings("rawtypes")
  @Override
  public @NotNull DeadeyeJsonAdapter getJsonAdapter() {
    return new JsonAdapterImpl();
  }

  private static class JsonAdapterImpl implements DeadeyeJsonAdapter<HubTargetData> {

    static final int DATA_LENGTH = 5;

    private static final JsonReader.Options OPTIONS =
        JsonReader.Options.of("id", "sn", "v", "ep", "r", "d");

    @Override
    public HubTargetData fromJson(BufferedSource source) throws IOException {
      JsonReader reader = JsonReader.of(source);
      String id = null;
      int serial = -1;
      boolean valid = false;
      double errorPixels = 0.0;
      double range = 0.0;
      List<Rect> targets = new ArrayList<>();

      reader.beginObject();
      while (reader.hasNext()) {
        switch (reader.selectName(OPTIONS)) {
          case 0:
            id = reader.nextString();
            break;
          case 1:
            serial = reader.nextInt();
            break;
          case 2:
            valid = reader.nextBoolean();
            break;
          case 3:
            errorPixels = reader.nextDouble();
            break;
          case 4:
            range = reader.nextDouble();
            break;
          case 5:
            reader.beginArray();
            while (reader.hasNext()) {
              int[] data = new int[DATA_LENGTH];
              reader.beginArray();
              for (int i = 0; i < DATA_LENGTH; i++) {
                data[i] = reader.nextInt();
              }
              reader.endArray();
              // bb.x, bb.y, bb.width, bb.height, area
              Point topLeft = new Point(data[0], data[1]);
              Point bottomRight = new Point(data[0] + data[2], data[1] + data[3]);
              targets.add(new Rect(topLeft, bottomRight, data[4]));
            }
            reader.endArray();
            break;
          default:
            throw new IllegalStateException("Unexpected value: " + reader.selectName(OPTIONS));
        }
      }
      reader.endObject();
      return new HubTargetData(
          Objects.requireNonNull(id), serial, valid, errorPixels, range, targets);
    }

    @Override
    public String toJson(HubTargetData targetData) throws IOException {
      Buffer buffer = new Buffer();
      JsonWriter writer = JsonWriter.of(buffer);
      writer.beginObject();
      writer.name("id").value(targetData.id);
      writer.name("sn").value(targetData.serial);
      writer.name("v").value(targetData.valid);

      writer.name("d").beginArray();
      for (Rect t : targetData.targets) {
        writer.beginArray();
        writer.value(t.topLeft.x).value(t.topLeft.y);
        writer.value(t.width()).value(t.height());
        writer.value(t.contourArea);
        writer.endArray();
      }
      writer.endArray();
      writer.endObject();
      return buffer.readUtf8();
    }
  }
}
