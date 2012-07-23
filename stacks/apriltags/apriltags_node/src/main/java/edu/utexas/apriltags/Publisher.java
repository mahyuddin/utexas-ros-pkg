package edu.utexas.april;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;
import org.ros.message.sensor_msgs;

/**
 * @author piyushk@gmail.com (Piyush Khandelwal)
 */
public class Listener extends AbstractNodeMain {

  @Override
  public GraphName getDefaultNodeName() {
    return new GraphName("apriltags_publisher");
  }

  @Override
  public void onStart(ConnectedNode connectedNode) {
    
    final Log log = connectedNode.getLog();
    log.info("Waiting for camera info message");
    boolean camera_info_received = false;

    Subscriber<sensor_msgs.Image> image_subscriber =
        node.newSubscriber("usb_cam/image_raw", sensor_msgs.Image._TYPE);
    image_subscriber.addMessageListener(new MessageListener<sensor_msgs.Image>() {
      @Override
      public void onNewMessage(sensor_msgs.Image message) {
        if (!camera_info_received) {
          return;
        }
        log.info("success!!");
      }
    });

    Subscriber<sensor_msgs.CameraInfo> info_subscriber =
        node.newSubscriber("usb_cam/camera_info", sensor_msgs.CameraInfo._TYPE);
    image_subscriber.addMessageListener(new MessageListener<sensor_msgs.CameraInfo>() {
      @Override
      public void onNewMessage(sensor_msgs.CameraInfo message) {
        log.info("Camera info message received");
        camera_info_received = true;
      }
    });
  }
}
