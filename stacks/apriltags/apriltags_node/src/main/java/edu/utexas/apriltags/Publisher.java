package edu.utexas.april;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;

/**
 * @author piyushk@gmail.com (Piyush Khandelwal)
 */
public class Publisher extends AbstractNodeMain {

  private boolean camera_info_received = false;

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("apriltags_publisher");
  }

  @Override
  public void onStart(ConnectedNode node) {
    
    final Log log = node.getLog();
    log.info("Waiting for camera info message");

    Subscriber<sensor_msgs.Image> image_subscriber =
        node.newSubscriber("image_raw", sensor_msgs.Image._TYPE);
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
        node.newSubscriber("camera_info", sensor_msgs.CameraInfo._TYPE);
    info_subscriber.addMessageListener(new MessageListener<sensor_msgs.CameraInfo>() {
      @Override
      public void onNewMessage(sensor_msgs.CameraInfo message) {
        log.info("Camera info message received");
        camera_info_received = true;
      }
    });
  }
}
