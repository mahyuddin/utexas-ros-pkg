import org.apache.commons.logging.Log;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.message.MessageListener;

// The following classed are automatically generaged and 
// are located in: ~/.ros/rosjava/lib/org.ros.rosjava.lightswitch_node-0.0.0.jar
import org.ros.message.lightswitch_node.AmbientLight;
import org.ros.message.lightswitch_node.LightSwitchCmd;

public class LightSwitchApp implements NodeMain {

  @Override
  public GraphName getDefaultNodeName() {
    // The parameter is the node's name.
    return new GraphName("lightswitch_app/LightSwitchApp");
  }

  private String printAmbientLight(AmbientLight message) {
    StringBuffer sb = new StringBuffer();
    sb.append("AmbientLight:\n");
	sb.append("\theader:\n");
	sb.append("\t\tseq = "         + message.header.seq + "\n");
        sb.append("\t\tstamp.secs = "  + message.header.stamp.secs + "\n");
        sb.append("\t\tstamp.nsecs = " + message.header.stamp.nsecs + "\n");
        sb.append("\t\tframe_id = "    + message.header.frame_id + "\n");
        sb.append("\tlightLevel = : "  + message.lightLevel + "\n");
	return sb.toString();
  }

  @Override
  public void onStart(final Node node) {
    final Publisher<LightSwitchCmd> publisher =
        node.newPublisher("lightswitch_cmd", "lightswitch_node/LightSwitchCmd");
    final Log log = node.getLog();

    log.info("Subscribing to AmbientLight messages.");
    Subscriber<AmbientLight> subscriber = node.newSubscriber("lightLevel", "lightswitch_node/AmbientLight");
    subscriber.addMessageListener(new MessageListener<AmbientLight>() {
      @Override
      public void onNewMessage(AmbientLight message) {
        log.info("Received AmbientLight message:\n" + printAmbientLight(message));
      }
    });


    log.info("Sleeping for 1s before starting.");
    try { Thread.sleep(1000); } catch(InterruptedException ie) { ie.printStackTrace(); }
    node.executeCancellableLoop(new CancellableLoop() {

      @Override
      protected void setup() {
      }

      @Override
      protected void loop() throws InterruptedException {
	/*Twist cmd = new Twist();
        log.info("Moving robot forward.");
	cmd.linear.x = 0.2; // move forward 0.2m/s for 1s
        publisher.publish(cmd);
        Thread.sleep(1000);

	publisher.publish(new Twist()); // pause for 1s
	Thread.sleep(1000);

	log.info("Moving robot backward.");
	cmd = new Twist();
	cmd.linear.x = -0.2; // move backwards 0.2m/s for 1s
	publisher.publish(cmd);
	Thread.sleep(1000);

	publisher.publish(new Twist()); // pause for 1s*/
	Thread.sleep(1000);
      }
    });
  }

  @Override
  public void onShutdown(Node node) {
  }

  @Override
  public void onShutdownComplete(Node node) {
  }
}
