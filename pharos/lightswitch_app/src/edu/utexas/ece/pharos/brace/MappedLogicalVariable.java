package edu.utexas.ece.pharos.brace;

/**
 * A mapped logical variable is one that is associated with
 * a physical variable.
 * 
 * @author Chien-Liang Fok
 * @param <V> The variable's data type.
 */
public interface MappedLogicalVariable<V> {
	
	/**
	 * Obtains the physical value of the mapped logical variable
	 * at the specified time with the specified max delta.
	 * 
	 * @param targetTime The ideal time in which to report the value of the 
	 * physical variable.
	 * @param delta The maximum difference between the targetTime and the
	 * actual time reported.
	 * @return The value of the physical variable at the specified time interval,
	 * or null of no such measurement was taken.
	 */
	public abstract PhysicalValue<V> getValue(long targetTime, long delta);
}
