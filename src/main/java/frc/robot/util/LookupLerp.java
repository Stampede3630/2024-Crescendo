package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Comparator;
import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

public abstract class LookupLerp<K extends Comparable<K>, V extends Comparable<V>> {
    private final NavigableMap<K, V> navigableMap;

    public LookupLerp() {
        if (comparator() != null)
            navigableMap = new TreeMap<>(comparator());
        else
            navigableMap = new TreeMap<>();
    }

    protected Comparator<K> comparator() {
        return null;
    }

    public void put(K key, V value) {
        navigableMap.put(key, value);
    }

    public final V get(K key) {
        if (navigableMap.containsKey(key)) {
            return navigableMap.get(key);
        }

        Map.Entry<K, V> top = navigableMap.ceilingEntry(key);
        Map.Entry<K, V> bottom = navigableMap.floorEntry(key);

        if (top == null) { // key is greater than top key
            top = navigableMap.lastEntry(); // get the last one
            bottom = navigableMap.lowerEntry(top.getKey()); // get the 2nd to last one
        } else if (bottom == null) { // key is less than bottom key
            bottom = navigableMap.firstEntry(); // get the first one
            top = navigableMap.higherEntry(bottom.getKey()); // get the 2nd one
        }

        //        navigableMap.put(key, value);
        return lerp(bottom.getKey(), bottom.getValue(), top.getKey(), top.getValue(), key);
    }


    protected abstract V lerp(K x1, V y1, K x2, V y2, K key);
}
