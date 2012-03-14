package edu.utexas.ece.pharos.demo.homeAutomation;

import edu.utexas.ece.pharos.brace.CPSPredicate;
import edu.utexas.ece.pharos.utils.Logger;

/**
 * A predicate that is true if the room is bright.
 * 
 * @author Chien-Liang Fok
 */
public class CPSPredicateRoomBright implements CPSPredicate {

	public static final int BRIGHT_THRESHOLD = 100;
	
	private MLVAmbientLight mlvAmbientLight;
	
	public CPSPredicateRoomBright(MLVAmbientLight mlvAmbientLight) {
		this.mlvAmbientLight = mlvAmbientLight;
	}
	
	@Override
	public boolean evaluate() {
		StringBuffer sb = new StringBuffer("Evaluating whether room is bright.\n");
		sb.append("\t- [" + System.currentTimeMillis() + "] Getting ambient light value...\n");
		int lightValue = mlvAmbientLight.getValue().getValue();
		sb.append("\t- [" + System.currentTimeMillis() + "] Got ambient light value " + lightValue + "...\n");
		
		boolean result = lightValue > BRIGHT_THRESHOLD;
		sb.append("\t- [" + System.currentTimeMillis() + "] Result = " + result + "...\n");
		
		Logger.log(sb.toString());
		
		return result;
	}
	
	public String toString() {
		return getClass().getName();
	}
}
