package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.kHorizonFov;

import com.squareup.moshi.JsonReader;
import com.squareup.moshi.JsonWriter;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
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
  static int kFrameVerticalCenter = Integer.MAX_VALUE;
  private final double errorPixels;
  private final double range;
  private final Logger logger = LoggerFactory.getLogger(this.getClass());
  public boolean evenNumTargets = false;

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
    // return valid && targets.size() > 2;
    return valid && !targets.isEmpty();
  }

  public boolean isRangingValid() {
    return valid && targets.size() > 2;
  }

  /**
   * Return the distance from the center of the Hub target group to the center of the camera frame.
   * You should check {@link #isValid()} before calling this method.
   *
   * @return the number of pixels, positive if ..., negative if ...
   * @throws IndexOutOfBoundsException if the list of targets is empty
   */
  public double getErrorPixels() {
    int minX = targets.get(0).bottomRight.x;
    int maxX = targets.get(targets.size() - 1).topLeft.x;
    return (maxX + minX) / 2.0 - kFrameCenter;
  }

  public double getErrorPixelsJetson() {
    return errorPixels;
  }

  public double getVerticalOffsetPixels() {
      int maxY = targets.get(Math.round(targets.size() / 2)).topLeft.y;
      return maxY;
  }
  public double getVerticalOffsetRadians() {
    return VisionConstants.kHorizonFov * getVerticalOffsetPixels() / (kFrameVerticalCenter * 2); //FIXME kVerticalFov?
  }

  /**
   * Return the angle from the center of the Hub target group to the center of the camera frame. You
   * should check {@link #isValid()} before calling this method.
   *
   * @return the angle in Radians
   * @throws IndexOutOfBoundsException if the list of targets is empty
   */
  public double getErrorRadians() {
    return -VisionConstants.kVerticalFov * getErrorPixels() / (kFrameCenter * 2); //FIXME kHorizonFov?
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

  public int testGetTargetsPixelWidth() {
    int pixelWidth;

    if (targets.size() % 2 == 1) {
      Rect leftTarget = targets.get((targets.size() - 1) / 2 - 1);
      Rect rightTarget = targets.get((targets.size() - 1) / 2 + 1);

      pixelWidth = rightTarget.bottomRight.x - leftTarget.topLeft.x;
      evenNumTargets = false;
    } else {
      Rect leftTarget = targets.get(targets.size() / 2 - 2);
      Rect rightTarget = targets.get(targets.size() / 2 + 1);

      pixelWidth = rightTarget.topLeft.x - leftTarget.bottomRight.x;
      evenNumTargets = true;
    }

    return pixelWidth;
  }

  public boolean isNumTargetsEven() {
    return evenNumTargets;
  }

  public int getVerticalPixelHeight() {
    int pixelHeight;
    if (targets.size() % 2 == 1) {
      Rect centerTarget = targets.get((targets.size() - 1) / 2);
      Rect rightTarget = targets.get((targets.size() - 1) / 2 + 1);
      Rect leftTarget = targets.get((targets.size() - 1) / 2 - 1);

      pixelHeight =
          (rightTarget.bottomRight.y + leftTarget.bottomRight.y) / 2 - centerTarget.topLeft.y;
    } else {
      Rect rightCenterTarget = targets.get(targets.size() / 2);
      Rect rightTarget = targets.get(targets.size() / 2 + 1);
      Rect leftCenterTarget = targets.get(targets.size() / 2 - 1);
      Rect leftTarget = targets.get(targets.size() / 2 - 2);

      pixelHeight =
          ((rightTarget.bottomRight.y - rightCenterTarget.topLeft.y)
                  + (leftTarget.bottomRight.y - leftCenterTarget.topLeft.y))
              / 2;
    }
    return pixelHeight;
  }

  public double testGetDistance() {
    double pixelWidth;

    if (targets.size() % 2 == 1) { // odd # of targets
      Rect leftTarget = targets.get((targets.size() - 1) / 2 - 1);
      Rect rightTarget = targets.get((targets.size() - 1) / 2 + 1);

      pixelWidth = rightTarget.bottomRight.x - leftTarget.topLeft.x;

      double enclosedAngle = Math.toDegrees(kHorizonFov) * pixelWidth / (kFrameCenter * 2);
      return 25.25 / 2 / Math.tan(Math.toRadians(enclosedAngle / 2));
    } else { // even # of targets
      Rect leftTarget = targets.get(targets.size() / 2 - 2);
      Rect rightTarget = targets.get(targets.size() / 2 + 1);

      pixelWidth = rightTarget.topLeft.x - leftTarget.bottomRight.x;

      double enclosedAngle = Math.toDegrees(kHorizonFov) * pixelWidth / (kFrameCenter * 2);
      return 25.625 / 2 / Math.tan(Math.toRadians(enclosedAngle / 2));
    }
  }

  public double getGroundDistance() {
    if (!isRangingValid()) {
      return 2767;
    }
    double x = testGetDistance();
    return 0.00462384837384838 * Math.sqrt(x) - 2.70841658341659 * x + 518.807692307693;
    /*return Math.sqrt(
        Math.pow(testGetDistance(), 2)
            - Math.pow(VisionConstants.kTapeHeightIn - VisionConstants.kCameraHeight, 2))
    + VisionConstants.kUpperHubRadiusIn;*/
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
