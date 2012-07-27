package april.ros;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;
import org.ros.node.topic.Publisher;
import org.ros.node.parameter.ParameterTree;

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
  
  private String tf_str = "april.tag.Tag36h11";
  private TagFamily tf;
  private TagDetector detector;

  private double tagSize;
  private double tagVisMagnification;
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
  public static void projectionMatrixToTranslationVector(
      double M[][], geometry_msgs.Point point) {
    point.setX(M[0][3]);
    point.setY(-M[1][3]);
    point.setZ(-M[2][3]);
  }

  /** Compute tag orientation as a Quaternion. Tag orientation is computed from
   * the rotational component of the projection matrix. The methodology was
   * taken from http://en.wikipedia.org/wiki/Rotation_matrix#Quaternion 
   */
  public static void projectionMatrixToQuaternion(
      double M[][], geometry_msgs.Quaternion q) {

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

    q.setX(qx);
    q.setY(-qy);
    q.setZ(-qz);
    q.setW(qw);

    //Rotate the quaternion
  }

  /** Compute the entire tag pose from the projection matrix
   */
  public static void projectionMatrixToPoseMessage(
      double M[][], geometry_msgs.Pose message) {
    TagPublisher.projectionMatrixToTranslationVector(M, message.getPosition());
    TagPublisher.projectionMatrixToQuaternion(M, message.getOrientation());
  }

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("april_tag_publisher");
  }

  @Override
  public void onStart(final ConnectedNode node) {

    // Get logger to publish ROS log messages
    final Log log = node.getLog();

    // Create detector object
    this.tf = (TagFamily) ReflectUtil.createObject(this.tf_str);
    detector = new TagDetector(this.tf);

    // Get parameters from parameter server
    ParameterTree params = node.getParameterTree();

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
    this.tagSize = params.getDouble("~tag_size", 0.095);
    this.tagVisMagnification = params.getDouble("~tag_vis_magnification", 5);
    boolean useCameraInfo = params.getBoolean("~useCameraInfo", false);
    this.focalLengthX = params.getDouble("~focal_length_x", 485.6);
    this.focalLengthY = params.getDouble("~focal_length_y", this.focalLengthX);
    if (!useCameraInfo) {
      focalLengthAvailable = true;
      log.info("Focal length values fx = " + focalLengthX + 
               ", fy = " + focalLengthY);
    }

    // Create the publisher for the Tag Array
    final Publisher<april_msgs.TagPoseArray> publisher = 
      node.newPublisher("~tags", april_msgs.TagPoseArray._TYPE);

    // Create the publisher for rviz visualization
    final Publisher<visualization_msgs.MarkerArray> visPublisher = 
      node.newPublisher("visualization_marker_array", 
                        visualization_msgs.MarkerArray._TYPE);

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

        visualization_msgs.MarkerArray visArray = visPublisher.newMessage();
        java.util.List<visualization_msgs.Marker> markers = 
            visArray.getMarkers();

        // For each tag, add the pose to the tag list
        for (TagDetection d : detections) {
          double p0[] = d.interpolate(-1,-1);
          double p1[] = d.interpolate(1,-1);
          double p2[] = d.interpolate(1,1);
          double p3[] = d.interpolate(-1,1);
          double M[][] = CameraUtil.homographyToPose(
              focalLengthX, focalLengthY, tagSize, d.homography);

          // Copy all information to a TagPose message
          april_msgs.TagPose tag = node.getTopicMessageFactory().
              newFromType(april_msgs.TagPose._TYPE);

          // Tag id and hamming distance error
          tag.setId(d.id);
          tag.setHammingDistance(d.hammingDistance);

          // Image coordinates of Tag Detection
          java.util.List<geometry_msgs.Point32> imageCoordinates = 
            tag.getImageCoordinates();
          // p0
          geometry_msgs.Point32 pt0 = node.getTopicMessageFactory().
              newFromType(geometry_msgs.Point32._TYPE);
          pt0.setX((float)p0[0]);
          pt0.setY((float)p0[1]);
          pt0.setZ(0);
          imageCoordinates.add(pt0);
          // p1
          geometry_msgs.Point32 pt1 = node.getTopicMessageFactory().
              newFromType(geometry_msgs.Point32._TYPE);
          pt1.setX((float)p1[0]);
          pt1.setY((float)p1[1]);
          pt1.setZ(0);
          imageCoordinates.add(pt1);
          // p2
          geometry_msgs.Point32 pt2 = node.getTopicMessageFactory().
            newFromType(geometry_msgs.Point32._TYPE);
          pt2.setX((float)p2[0]);
          pt2.setY((float)p2[1]);
          pt2.setZ(0);
          imageCoordinates.add(pt2);
          // p3
          geometry_msgs.Point32 pt3 = node.getTopicMessageFactory().
            newFromType(geometry_msgs.Point32._TYPE);
          pt3.setX((float)p3[0]);
          pt3.setY((float)p3[1]);
          pt3.setZ(0);
          imageCoordinates.add(pt3);

          // Pose of the the tag
          TagPublisher.projectionMatrixToPoseMessage(M, tag.getPose());

          // Add tag to the array
          tags.add(tag);

          // Setup the corresponding visualization
          visualization_msgs.Marker marker = node.getTopicMessageFactory().
            newFromType(visualization_msgs.Marker._TYPE);
          marker.setHeader(message.getHeader());
          marker.setNs("tag" + d.id);
          marker.setId(0);
          marker.setAction(visualization_msgs.Marker.ADD);
          marker.setPose(tag.getPose());

          // Use Line strip
          marker.setType(visualization_msgs.Marker.LINE_STRIP);
          marker.getScale().setX(0.02);
          java.util.List<geometry_msgs.Point> points = marker.getPoints();
          // p0
          geometry_msgs.Point mpt0 = node.getTopicMessageFactory().
              newFromType(geometry_msgs.Point._TYPE);
          mpt0.setX(tagVisMagnification * tagSize/2);
          mpt0.setY(tagVisMagnification * tagSize/2);
          mpt0.setZ(0);
          points.add(mpt0);
          // p1
          geometry_msgs.Point mpt1 = node.getTopicMessageFactory().
              newFromType(geometry_msgs.Point._TYPE);
          mpt1.setX(tagVisMagnification * tagSize/2);
          mpt1.setY(-tagVisMagnification * tagSize/2);
          mpt1.setZ(0);
          points.add(mpt1);
          // p2
          geometry_msgs.Point mpt2 = node.getTopicMessageFactory().
            newFromType(geometry_msgs.Point._TYPE);
          mpt2.setX(-tagVisMagnification * tagSize/2);
          mpt2.setY(-tagVisMagnification * tagSize/2);
          mpt2.setZ(0);
          points.add(mpt2);
          // p3
          geometry_msgs.Point mpt3 = node.getTopicMessageFactory().
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
          markers.add(marker);

          visualization_msgs.Marker markerText = node.getTopicMessageFactory().
            newFromType(visualization_msgs.Marker._TYPE);
          markerText.setHeader(message.getHeader());
          markerText.setNs("tag" + d.id);
          markerText.setId(1);
          markerText.setType(visualization_msgs.Marker.TEXT_VIEW_FACING);
          markerText.setAction(visualization_msgs.Marker.ADD);
          markerText.setPose(tag.getPose());
          markerText.getScale().setZ(0.1);
          markerText.setText("Tag: " + d.id);
          markerText.getColor().setR(0.6f);
          markerText.getColor().setG(0.6f);
          markerText.getColor().setB(0.6f);
          markerText.getColor().setA(1);
          markers.add(markerText);
        }
        
        // Finally publish all detected tags
        publisher.publish(tagArray);
        visPublisher.publish(visArray);
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
