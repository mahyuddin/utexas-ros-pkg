package edu.utexas.ece.pharos.brace;

import edu.utexas.ece.pharos.utils.Logger;

/**
 * A handle to a CPS assertion.
 * 
 * @author Chien-Liang Fok
 *
 */
public class CPSAssertion {

	private CPSPredicate predicate;
	
	private boolean evaluated = false;
	private long initTime, maxLatency;
	private boolean failSilently = false;
	
	public CPSAssertion(CPSPredicate predicate, long maxLatency, boolean failSilently) {
		this.predicate = predicate;
		this.maxLatency = maxLatency;
		this.failSilently = failSilently;
		this.initTime = System.currentTimeMillis();
	}
	
	public boolean isEvaluated() {
		return evaluated;
	}
	
	public void evaluate() {
		if (!evaluated) {
			evaluated = true;
			if (System.currentTimeMillis() - initTime < maxLatency) {
				if (!predicate.evaluate()) {
					Logger.logErr("Assertion failed: " + predicate);
					System.exit(-1);
				}
			} else {
				if (!failSilently) {
					Logger.logErr("Assertion failed due to maxLatency violation.");
					System.exit(-1);
				}
			}
		}
	}
}
