package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.kHorizonFov;

import com.squareup.moshi.JsonReader;
import com.squareup.moshi.JsonWriter;
import edu.wpi.first.math.geometry.Rotation2d;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import okio.Buffer;
import okio.BufferedSource;
import org.jetbrains.annotations.NotNull;
import org.strykeforce.deadeye.DeadeyeJsonAdapter;
import org.strykeforce.deadeye.Point;
import org.strykeforce.deadeye.Rect;
import org.strykeforce.deadeye.TargetListTargetData;

public class HubTargetData extends TargetListTargetData {

  static int kFrameCenter = Integer.MAX_VALUE;

  public HubTargetData() {
    super();
  }

  public HubTargetData(@NotNull String id, int serial, boolean valid, @NotNull List<Rect> targets) {
    super(id, serial, valid, targets);
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

  @SuppressWarnings("rawtypes")
  @Override
  public @NotNull DeadeyeJsonAdapter getJsonAdapter() {
    return new JsonAdapterImpl();
  }

  private static class JsonAdapterImpl implements DeadeyeJsonAdapter<HubTargetData> {

    static final int DATA_LENGTH = 5;

    // json d field: bb.tl().x, bb.tl().y, bb.br().x, bb.br().y, center.x, center.y
    private static final JsonReader.Options OPTIONS = JsonReader.Options.of("id", "sn", "v", "d");

    @Override
    public HubTargetData fromJson(BufferedSource source) throws IOException {
      JsonReader reader = JsonReader.of(source);
      String id = null;
      int serial = -1;
      boolean valid = false;
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
      return new HubTargetData(Objects.requireNonNull(id), serial, valid, targets);
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
