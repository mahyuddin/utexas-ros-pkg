package april.ros;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.message.MessageFactory;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;
import org.ros.node.topic.Publisher;
import org.ros.node.parameter.ParameterTree;
import org.ros.rosjava_geometry.*;

import april.tag.*;
import april.util.*;
import april.jmat.*;

import java.awt.image.*;
import java.awt.Color;
import java.awt.color.*;
import java.awt.*;
import java.util.ArrayList;
import java.io.File;
import java.io.InputStream;
import java.io.IOException;
import java.io.ByteArrayInputStream;
import javax.imageio.ImageIO;

import java.lang.System;
import java.lang.Math;

public class TagPublisher extends AbstractNodeMain {

  private boolean focalLengthAvailable = false;
  
  private TagDetector detector;

  private double focalLengthX;
  private double focalLengthY;

  /**
   * Simple but inefficient conversion of Image message to Buffered Image.
   */
  public static BufferedImage messageToBufferedImage(
      sensor_msgs.Image message) {

    BufferedImage im = 
        new BufferedImage(message.getWidth(), 
                          message.getHeight(), 
                          BufferedImage.TYPE_INT_RGB);

    for (int x = 0; x < message.getWidth(); x++) {
      for (int y = 0; y < message.getHeight(); y++) {
        byte red = 
          message.getData().getByte((int) (y * message.getStep() + 3 * x));
        byte green = 
          message.getData().getByte((int) (y * message.getStep() + 3 * x + 1));
        byte blue = 
          message.getData().getByte((int) (y * message.getStep() + 3 * x + 2));
        int rgb = (red & 0xFF);
        rgb = (rgb << 8) + (green & 0xFF);
        rgb = (rgb << 8) + (blue & 0xFF);
        im.setRGB(x, y, rgb);
      }
    }

    return im;
  }

  /** 
   * Efficient but channel flipped conversion of Image message to Buffered
   * Image. The image message data stores data in 3 bytes as RGBRGBR...., 
   * which is not supported directly by BufferedImage. The only 3 byte 
   * representation supported by BufferedImage is 3BYTE_BGR. Since the tags are
   * unaffected by color, we flip the R and B channels in this efficient copy.
   */
  public static BufferedImage messageToBufferedImageFast(
      sensor_msgs.Image message) {

    int width = message.getWidth();
    int height = message.getHeight();

    DataBufferByte dataBuffer = 
      new DataBufferByte(message.getData().array(), 
                         message.getData().array().length);
    PixelInterleavedSampleModel sampleModel = 
      new PixelInterleavedSampleModel(
          DataBuffer.TYPE_BYTE, width, height, 3, 3*width, new int[] {2,1,0}); 
    WritableRaster raster = 
      Raster.createWritableRaster(sampleModel, dataBuffer, new Point(0,0));

    ColorSpace cs = ColorSpace.getInstance(ColorSpace.CS_sRGB);
    ColorModel colourModel = new ComponentColorModel(
        cs, false, false, Transparency.OPAQUE, DataBuffer.TYPE_BYTE);

    BufferedImage im = new BufferedImage(colourModel, raster, false, null);

    return im;
  }

  /** Compute tag location in the standard ROS frame convention for optical
   * frames. Tag location is computed from the translational component of
   * the projection matrix. 
   * To get more details about the frame convention, see REP 103 
   * (http://www.ros.org/reps/rep-0103.html)
   */
  public static Vector3 projectionMatrixToTranslationVector(double M[][]) {
    Vector3 point = new Vector3(M[0][3], -M[1][3], -M[2][3]);
    return point;
  }

  /** Compute tag orientation as a Quaternion. Tag orientation is computed from
   * the rotational component of the projection matrix. The methodology was
   * taken from http://en.wikipedia.org/wiki/Rotation_matrix#Quaternion 
   */
  public static Quaternion projectionMatrixToQuaternion(double M[][]) {

    double qx, qy, qz, qw;

    double t = M[0][0] + M[1][1] + M[2][2]; 
    if (t > 0) {
      double r = Math.sqrt(1+t);
      double s = 0.5f / r;
      qw = 0.5 * r;
      qx = (M[2][1] - M[1][2]) * s;
      qy = (M[0][2] - M[2][0]) * s;
      qz = (M[1][0] - M[0][1]) * s;
    } else {
      if (M[0][0] > M[1][1] && M[0][0] > M[2][2]) {
        double r = Math.sqrt(1.0f + M[0][0] - M[1][1] - M[2][2]);
        double s = 0.5f / r;
        qw = (M[2][1] - M[1][2]) * s;
        qx = 0.5f * r;
        qy = (M[0][1] + M[1][0]) * s;
        qz = (M[2][0] + M[0][2]) * s;
      } else if (M[1][1] > M[2][2]) {
        double r = Math.sqrt(1.0f + M[1][1] - M[0][0] - M[2][2]);
        double s = 0.5f / r;
        qw = (M[0][2] - M[2][0]) * s;
        qx = (M[0][1] + M[1][0]) * s;
        qy = 0.5f * r;
        qz = (M[1][2] + M[2][1]) * s;
      } else {
        double r = Math.sqrt(1.0f + M[2][2] - M[0][0] - M[1][1]);
        double s = 0.5f / r;
        qw = (M[1][0] - M[0][1]) * s;
        qx = (M[0][2] + M[2][0]) * s;
        qy = (M[1][2] + M[2][1]) * s;
        qz = 0.5f * r;
      }
    }

    return new Quaternion(qx, -qy, -qz, qw);
  }

  /** Compute the entire tag pose from the projection matrix
   */
  public static Transform projectionMatrixToTransform(double M[][]) {
    return new Transform(TagPublisher.projectionMatrixToTranslationVector(M),
        TagPublisher.projectionMatrixToQuaternion(M));
  }

  public static void detectionToTagPose(TagDetection d, Transform transform, 
      april_msgs.TagPose tag, MessageFactory msgFactory) {

    // Tag id and hamming distance error
    tag.setId(d.id);
    tag.setHammingDistance(d.hammingDistance);

    // Image coordinates of Tag Detection
    double p0[] = d.interpolate(-1,-1);
    double p1[] = d.interpolate(1,-1);
    double p2[] = d.interpolate(1,1);
    double p3[] = d.interpolate(-1,1);
    java.util.List<geometry_msgs.Point32> imageCoordinates = 
      tag.getImageCoordinates();
    // p0
    geometry_msgs.Point32 pt0 = msgFactory.
      newFromType(geometry_msgs.Point32._TYPE);
    pt0.setX((float)p0[0]);
    pt0.setY((float)p0[1]);
    pt0.setZ(0);
    imageCoordinates.add(pt0);
    // p1
    geometry_msgs.Point32 pt1 = msgFactory.
      newFromType(geometry_msgs.Point32._TYPE);
    pt1.setX((float)p1[0]);
    pt1.setY((float)p1[1]);
    pt1.setZ(0);
    imageCoordinates.add(pt1);
    // p2
    geometry_msgs.Point32 pt2 = msgFactory.
      newFromType(geometry_msgs.Point32._TYPE);
    pt2.setX((float)p2[0]);
    pt2.setY((float)p2[1]);
    pt2.setZ(0);
    imageCoordinates.add(pt2);
    // p3
    geometry_msgs.Point32 pt3 = msgFactory.
      newFromType(geometry_msgs.Point32._TYPE);
    pt3.setX((float)p3[0]);
    pt3.setY((float)p3[1]);
    pt3.setZ(0);
    imageCoordinates.add(pt3);

    // Pose of the the tag
    transform.toPoseMessage(tag.getPose());
  }

  public static void tagPoseToVisualizationMarker(april_msgs.TagPose tag,
      visualization_msgs.Marker marker, double tagVisMagnification, 
      double tagSize, MessageFactory msgFactory) {
    marker.setNs("tag" + tag.getId());
    marker.setId(0);
    marker.setAction(visualization_msgs.Marker.ADD);
    marker.setPose(tag.getPose());

    // Use Line strip
    marker.setType(visualization_msgs.Marker.LINE_STRIP);
    marker.getScale().setX(0.02);
    java.util.List<geometry_msgs.Point> points = marker.getPoints();
    // p0
    geometry_msgs.Point mpt0 = msgFactory.
      newFromType(geometry_msgs.Point._TYPE);
    mpt0.setX(tagVisMagnification * tagSize/2);
    mpt0.setY(tagVisMagnification * tagSize/2);
    mpt0.setZ(0);
    points.add(mpt0);
    // p1
    geometry_msgs.Point mpt1 = msgFactory.
      newFromType(geometry_msgs.Point._TYPE);
    mpt1.setX(tagVisMagnification * tagSize/2);
    mpt1.setY(-tagVisMagnification * tagSize/2);
    mpt1.setZ(0);
    points.add(mpt1);
    // p2
    geometry_msgs.Point mpt2 = msgFactory.
      newFromType(geometry_msgs.Point._TYPE);
    mpt2.setX(-tagVisMagnification * tagSize/2);
    mpt2.setY(-tagVisMagnification * tagSize/2);
    mpt2.setZ(0);
    points.add(mpt2);
    // p3
    geometry_msgs.Point mpt3 = msgFactory.
      newFromType(geometry_msgs.Point._TYPE);
    mpt3.setX(-tagVisMagnification * tagSize/2);
    mpt3.setY(+tagVisMagnification * tagSize/2);
    mpt3.setZ(0);
    points.add(mpt3);
    // p4
    points.add(mpt0);
    marker.getColor().setR(0.6f);
    marker.getColor().setG(0.6f);
    marker.getColor().setB(0.6f);
    marker.getColor().setA(1);

    // Alternatively, use mesh (currently not working)
    /* marker.setType(visualization_msgs.Marker.MESH_RESOURCE); */
    /* marker.getScale().setX(tagSize * 5); */
    /* marker.getScale().setY(tagSize * 5); */
    /* marker.setMeshResource("package://april_tags_node/meshes/tag.stl"); */
    /* marker.setMeshUseEmbeddedMaterials(true); */
  }

  public static void tagPoseToVisualizationText(april_msgs.TagPose tag,
      visualization_msgs.Marker marker) {
    marker.setNs("tag" + tag.getId());
    marker.setId(1);
    marker.setType(visualization_msgs.Marker.TEXT_VIEW_FACING);
    marker.setAction(visualization_msgs.Marker.ADD);
    marker.setPose(tag.getPose());
    marker.getScale().setZ(0.1);
    marker.setText("Tag: " + tag.getId());
    marker.getColor().setR(0.6f);
    marker.getColor().setG(0.6f);
    marker.getColor().setB(0.6f);
    marker.getColor().setA(1);
  }

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("april_tag_publisher");
  }

  @Override
  public void onStart(final ConnectedNode node) {

    // Get logger to publish ROS log messages
    final Log log = node.getLog();

    // Get parameters from parameter server
    ParameterTree params = node.getParameterTree();

    // Create detector object
    final String tagFamilyStr = params.getString("~tag_family", "april.tag.Tag36h11"); 
    final TagFamily tagFamily = (TagFamily) ReflectUtil.createObject(tagFamilyStr);
    detector = new TagDetector(tagFamily);

    // Tag Detector Parameters
    double segSigma = params.getDouble("~seg_sigma", detector.segSigma); 
    double sigma = params.getDouble("~sigma", detector.sigma); 
    double minMag = params.getDouble("~min_mag", detector.minMag); 
    double maxEdgeCost = 
      params.getDouble("~max_edge_cost", detector.maxEdgeCost); 
    double magThresh = params.getDouble("~mag_thresh", detector.magThresh); 
    double thetaThresh = 
      params.getDouble("~theta_thresh", detector.thetaThresh);
    int errorBits = params.getInteger("~error_bits", 1);
    int weightScale = 
      params.getInteger("~weight_scale", detector.WEIGHT_SCALE);
    boolean segDecimate = 
      params.getBoolean("~seg_decimate", detector.segDecimate);
    boolean debug = params.getBoolean("~debug", false);

    detector.debug = debug;
    detector.sigma = sigma;
    detector.segSigma = segSigma;
    detector.segDecimate = segDecimate; 
    detector.minMag = minMag;
    detector.maxEdgeCost = maxEdgeCost;
    detector.magThresh = magThresh;
    detector.thetaThresh = thetaThresh;
    detector.WEIGHT_SCALE = weightScale;

    // Tag pose estimation parameters
    final double tagSize = params.getDouble("~tag_size", 0.095);
    boolean useCameraInfo = params.getBoolean("~use_camera_info", true);
    this.focalLengthX = params.getDouble("~focal_length_x", 485.6);
    this.focalLengthY = params.getDouble("~focal_length_y", this.focalLengthX);
    if (!useCameraInfo) {
      focalLengthAvailable = true;
      log.info("Focal length values fx = " + focalLengthX + 
               ", fy = " + focalLengthY);
    }

    // ROS specific parameters
    final boolean publishVisualization = 
        params.getBoolean("~publish_visualization", true);
    final double tagVisMagnification = params.getDouble("~tag_vis_magnification", 1);
    final boolean broadcastTf = 
        params.getBoolean("~broadcast_tf", true);

    // Create the publisher for the Tag Array
    final Publisher<april_msgs.TagPoseArray> publisher = 
      node.newPublisher("~tags", april_msgs.TagPoseArray._TYPE);

    // Create the publisher for rviz visualization
    final Publisher<visualization_msgs.MarkerArray> visPublisher;
    visPublisher = node.newPublisher("visualization_marker_array", 
          visualization_msgs.MarkerArray._TYPE);

    // Create publisher for broadcastin tf messages
    final Publisher<tf.tfMessage> tfPublisher;
    tfPublisher = node.newPublisher("tf", tf.tfMessage._TYPE);

    // Subscribe to the image message    
    Subscriber<sensor_msgs.Image> image_subscriber =
        node.newSubscriber("image_raw", sensor_msgs.Image._TYPE);
    image_subscriber.addMessageListener(
        new MessageListener<sensor_msgs.Image>() {
      @Override
      public void onNewMessage(sensor_msgs.Image message) {

        // Do not process image messages if still waiting for camera info msg
        if (!focalLengthAvailable) {
          log.debug("Waiting for camera info message");
          return;
        }

        // Obtain Buffered Image from the ROS image message
        BufferedImage im = TagPublisher.messageToBufferedImageFast(message);
        double[] size = new double[] {im.getWidth()/2.0, im.getHeight()/2.0};

        // Detect tags!
        ArrayList<TagDetection> detections = detector.process(im, size);

        // Setup messages to publish detection information
        april_msgs.TagPoseArray tagArray = publisher.newMessage();
        tagArray.setHeader(message.getHeader());
        java.util.List<april_msgs.TagPose> tags = tagArray.getTags();

        visualization_msgs.MarkerArray visArray;
        java.util.List<visualization_msgs.Marker> markers;
        visArray = visPublisher.newMessage();
        markers = visArray.getMarkers();

        tf.tfMessage tfMsg = tfPublisher.newMessage();
        java.util.List<geometry_msgs.TransformStamped> transforms = 
            tfMsg.getTransforms();

        // For each tag, add the pose to the tag list
        for (TagDetection d : detections) {
          double M[][] = CameraUtil.homographyToPose(
              focalLengthX, focalLengthY, tagSize, d.homography);

          // Compute the transformation for this detection
          Transform transform = TagPublisher.projectionMatrixToTransform(M);

          // Construct the TagPose message to transmit 
          april_msgs.TagPose tag = node.getTopicMessageFactory().
              newFromType(april_msgs.TagPose._TYPE);
          TagPublisher.detectionToTagPose(d, transform, tag,
              node.getTopicMessageFactory());
          tags.add(tag);

          // Publish visualization for rviz
          if (publishVisualization) {
            visualization_msgs.Marker marker = node.getTopicMessageFactory().
              newFromType(visualization_msgs.Marker._TYPE);
            marker.setHeader(message.getHeader());
            TagPublisher.tagPoseToVisualizationMarker(tag, marker, 
                tagVisMagnification, tagSize, node.getTopicMessageFactory());
            markers.add(marker);

            visualization_msgs.Marker markerText = node.getTopicMessageFactory().
              newFromType(visualization_msgs.Marker._TYPE);
            markerText.setHeader(message.getHeader());
            TagPublisher.tagPoseToVisualizationText(tag, markerText);
            markers.add(markerText);
          }

          // Broadcast tf 
          if (broadcastTf) {
            geometry_msgs.TransformStamped transformMsg = 
              node.getTopicMessageFactory().
              newFromType(geometry_msgs.TransformStamped._TYPE);
            transformMsg.setHeader(message.getHeader());
            transformMsg.setChildFrameId(tagFamilyStr + "." + tag.getId());
            transform.toTransformMessage(transformMsg.getTransform());
            transforms.add(transformMsg);
          }
        }
        
        // Finally publish all everything for this frame
        publisher.publish(tagArray);
        if (publishVisualization) {
          visPublisher.publish(visArray);
        }
        if (broadcastTf) {
          tfPublisher.publish(tfMsg);
        }
      }
    }); /* end addMessageListener */

    // Subscribe to camera info message if necessary
    if (useCameraInfo) {
      Subscriber<sensor_msgs.CameraInfo> info_subscriber =
        node.newSubscriber("camera_info", sensor_msgs.CameraInfo._TYPE);
      info_subscriber.addMessageListener(
          new MessageListener<sensor_msgs.CameraInfo>() {
        @Override
        public void onNewMessage(sensor_msgs.CameraInfo message) {
          if (!focalLengthAvailable) {
            log.info("Camera Info message received");
            double[] k = message.getK();
            if (k.length != 9) {
              log.error("Camera info message has malformed or non-existent " +
                "intrinsic camera matrix; unable to retrieve focal length. " +
                "Using default values!");
            } else {
              focalLengthX = k[0];
              focalLengthY = k[4];
              log.info("Focal length selected from camera info message");
            }
            log.info("Focal length values fx = " + focalLengthX + 
                     ", fy = " + focalLengthY);
            focalLengthAvailable = true;
          }
        }
      });
    } /* end useCameraInfo */

  }
}
