package edu.utexas.ece.pharos.brace;

public interface MappedLogicalVariable<V> {
	
	public abstract PhysicalValue<V> getValue();
	
	public abstract PhysicalValue<V> getValueAtTime(long timestamp);
}
