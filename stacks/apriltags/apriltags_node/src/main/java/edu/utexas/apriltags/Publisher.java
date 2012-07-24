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

public class Publisher extends AbstractNodeMain {

  private boolean camera_info_received = false;
  
  private String tf_str = "april.tag.Tag36h11";
  private TagFamily tf;
  private TagDetector detector;

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("apriltags_publisher");
  }

  @Override
  public void onStart(ConnectedNode node) {

    // Create detector object
    this.tf = (TagFamily) ReflectUtil.createObject(this.tf_str);
    detector = new TagDetector(this.tf);

    // Get parameters from parameter server
    ParameterTree params = node.getParameterTree();
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

    final Log log = node.getLog();
    // Subscribe to appropriate messages
    log.info("Waiting for camera info message");

    Subscriber<sensor_msgs.Image> image_subscriber =
        node.newSubscriber("image_raw", sensor_msgs.Image._TYPE);
    image_subscriber.addMessageListener(new MessageListener<sensor_msgs.Image>() {
      @Override
      public void onNewMessage(sensor_msgs.Image message) {
        if (!camera_info_received) {
          return;
        }

        /* BufferedImage im = new BufferedImage(message.getWidth(), message.getHeight(), BufferedImage.TYPE_INT_RGB); */
        /* // Convert sensor_msgs.Image to BufferedImage */
        /* for (int x = 0; x < message.getWidth(); x++) { */
        /*   for (int y = 0; y < message.getHeight(); y++) { */
        /*     byte red =  */
        /*         message.getData().getByte((int) (y * message.getStep() + 3 * x)); */
        /*     byte green =  */
        /*         message.getData().getByte((int) (y * message.getStep() + 3 * x + 1)); */
        /*     byte blue =  */
        /*         message.getData().getByte((int) (y * message.getStep() + 3 * x + 2)); */
        /*     int rgb = (red & 0xFF); */
        /*     rgb = (rgb << 8) + (green & 0xFF); */
        /*     rgb = (rgb << 8) + (blue & 0xFF); */
        /*     im.setRGB(x, y, rgb); */
        /*   } */
        /* } */

        int width = message.getWidth();
        int height = message.getHeight();
        DataBufferByte dataBuffer = new DataBufferByte(message.getData().array(), message.getData().array().length);
        PixelInterleavedSampleModel sampleModel
          = new PixelInterleavedSampleModel(DataBuffer.TYPE_BYTE, width, height, 3, 3*width, new int[] {2,1,0});        
        ColorSpace cs = ColorSpace.getInstance(ColorSpace.CS_sRGB);
        ColorModel colourModel = new ComponentColorModel(cs, false, false, Transparency.OPAQUE, DataBuffer.TYPE_BYTE);
        WritableRaster raster = Raster.createWritableRaster(sampleModel, dataBuffer, new Point(0,0));
        BufferedImage im = new BufferedImage(colourModel, raster, false, null);

        // Get detections
        ArrayList<TagDetection> detections = detector.process(im, new double[] {im.getWidth()/2.0, im.getHeight()/2.0});

        // Print number of detections
        log.info("Saw " + detections.size() + " markers");

      }
    });

    Subscriber<sensor_msgs.CameraInfo> info_subscriber =
        node.newSubscriber("camera_info", sensor_msgs.CameraInfo._TYPE);
    info_subscriber.addMessageListener(new MessageListener<sensor_msgs.CameraInfo>() {
      @Override
      public void onNewMessage(sensor_msgs.CameraInfo message) {
        if (!camera_info_received) {
          log.info("Camera information received");
          camera_info_received = true;
        }
      }
    });
  }
}
