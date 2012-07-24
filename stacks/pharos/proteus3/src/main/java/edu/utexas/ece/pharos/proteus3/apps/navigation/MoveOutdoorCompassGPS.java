package edu.utexas.ece.pharos.proteus3.apps.navigation;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;

import edu.utexas.ece.pharos.proteus3.sensors.CompassBuffer;
import edu.utexas.ece.pharos.proteus3.sensors.GPSBuffer;
import edu.utexas.ece.pharos.proteus3.navigate.NavigateCompassGPS;

// Import the messages
import proteus3_compass.CompassMsg;
import proteus3_gps.GPSMsg;

/**
 * Moves a robot to a particular latitude and longitude coordinate
 * using information from compass and GPS sensors.
 *
 * @author Chien-Liang Fok
 */
public class MoveOutdoorCompassGPS extends AbstractNodeMain {

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("proteus3/move_outdoor_compass_gps");
  }

  @Override
  public void onStart(ConnectedNode connectedNode) {
    final Log log = connectedNode.getLog();
    
    CompassBuffer compassBuffer = new CompassBuffer(log);
    GPSBuffer gpsBuffer = new GPSBuffer(log);

    Subscriber<CompassMsg> compassSubscriber = connectedNode.newSubscriber("/compass/measurement", CompassMsg._TYPE);
    compassSubscriber.addMessageListener(compassBuffer); 
    
    Subscriber<GPSMsg> gpsSubscriber = connectedNode.newSubscriber("/gps/measurement", GPSMsg._TYPE);
    gpsSubscriber.addMessageListener(gpsBuffer); 

    NavigateCompassGPS navCompGPS = new NavigateCompassGPS(compassSubscriber, gpsSubscriber);  
  }
}
