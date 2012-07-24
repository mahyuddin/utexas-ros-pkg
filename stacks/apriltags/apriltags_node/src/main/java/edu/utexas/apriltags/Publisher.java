package edu.utexas.april;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;
import org.ros.node.parameter.ParameterTree;

import april.tag.*;
import april.util.*;

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

public class Publisher extends AbstractNodeMain {

  private boolean focal_length_available = false;
  
  private String tf_str = "april.tag.Tag36h11";
  private TagFamily tf;
  private TagDetector detector;

  private double tagSize;
  private double focalLengthX;
  private double focalLengthY;

  public static BufferedImage messageToBufferedImage(sensor_msgs.Image message) {
    BufferedImage im = new BufferedImage(message.getWidth(), message.getHeight(), BufferedImage.TYPE_INT_RGB);
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

  public static BufferedImage messageToBufferedImageFast(sensor_msgs.Image message) {
    int width = message.getWidth();
    int height = message.getHeight();
    DataBufferByte dataBuffer = new DataBufferByte(message.getData().array(), message.getData().array().length);
    PixelInterleavedSampleModel sampleModel
      = new PixelInterleavedSampleModel(DataBuffer.TYPE_BYTE, width, height, 3, 3*width, new int[] {2,1,0});        
    ColorSpace cs = ColorSpace.getInstance(ColorSpace.CS_sRGB);
    ColorModel colourModel = new ComponentColorModel(cs, false, false, Transparency.OPAQUE, DataBuffer.TYPE_BYTE);
    WritableRaster raster = Raster.createWritableRaster(sampleModel, dataBuffer, new Point(0,0));
    BufferedImage im = new BufferedImage(colourModel, raster, false, null);
    return im;
  }

  public static geometry_msgs.Pose projectionMatrixToPoseMessage(double M[][]) {
    

    return message;
  }

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("apriltags_publisher");
  }

  @Override
  public void onStart(ConnectedNode node) {

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
    double maxEdgeCost = params.getDouble("~max_edge_cost", detector.maxEdgeCost); 
    double magThresh = params.getDouble("~mag_thresh", detector.magThresh); 
    double thetaThresh = params.getDouble("~theta_thresh", detector.thetaThresh);
    int errorBits = params.getInteger("~error_bits", 1);
    int weightScale = params.getInteger("~weight_scale", detector.WEIGHT_SCALE);
    boolean segDecimate = params.getBoolean("~seg_decimate", detector.segDecimate);
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
    boolean use_camera_info = params.getBoolean("~use_camera_info", false);
    this.focalLengthX = params.getDouble("~focal_length_x", 485.6);
    this.focalLengthY = params.getDouble("~focal_length_y", this.focalLengthX);
    if (!use_camera_info) {
      focal_length_available = true;
      log.info("Focal length values fx = " + focalLengthX + ", fy = " + focalLengthY);
    }

    // Subscribe to the image message    
    Subscriber<sensor_msgs.Image> image_subscriber =
        node.newSubscriber("image_raw", sensor_msgs.Image._TYPE);
    image_subscriber.addMessageListener(new MessageListener<sensor_msgs.Image>() {
      @Override
      public void onNewMessage(sensor_msgs.Image message) {
        if (!focal_length_available) {
          log.debug("Waiting for camera info message");
          return;
        }
        BufferedImage im = Publisher.messageToBufferedImageFast(message);
        ArrayList<TagDetection> detections = detector.process(im, new double[] {im.getWidth()/2.0, im.getHeight()/2.0});
        log.info("Num Detections: " + detections.size());

        // Publish information corresponding to every detection
        for (TagDetection d : detections) {
          double p0[] = d.interpolate(-1,-1);
          double p1[] = d.interpolate(1,-1);
          double p2[] = d.interpolate(1,1);
          double p3[] = d.interpolate(-1,1);
          double M[][] = CameraUtil.homographyToPose(focalLengthX, focalLengthY, tagSize, d.homography);
          System.out.println(M[0][0] + " " + M[0][1] + " " + M[0][2] + " " + M[0][3]);
          System.out.println(M[1][0] + " " + M[1][1] + " " + M[1][2] + " " + M[1][3]);
          System.out.println(M[2][0] + " " + M[2][1] + " " + M[2][2] + " " + M[2][3]);
        }
      }
    });

    // Subscribe to camera info message if necessary
    if (use_camera_info) {
      Subscriber<sensor_msgs.CameraInfo> info_subscriber =
        node.newSubscriber("camera_info", sensor_msgs.CameraInfo._TYPE);
      info_subscriber.addMessageListener(new MessageListener<sensor_msgs.CameraInfo>() {
        @Override
        public void onNewMessage(sensor_msgs.CameraInfo message) {
          if (!focal_length_available) {
            log.info("Camera Info message received");
            double[] k = message.getK();
            if (k.length != 9) {
              log.error("Camera info message has malformed or non-existent intrinsic camera matrix; unable to retrieve focal length. Using default values!");
            } else {
              focalLengthX = k[0];
              focalLengthY = k[4];
              log.info("Focal length selected from camera info message");
            }
            log.info("Focal length values fx = " + focalLengthX + ", fy = " + focalLengthY);
            focal_length_available = true;
          }
        }
      });
    } /* end use_camera_info */

  }
}
