package org.team100.robot;

/**
 * Keeps score, serves a web page describing the game state.
 * 
 * The general idea is to use "sensors" to produce collision events, and then
 * listen for them using a collision listener.
 * 
 * example:
 * 
 * https://github.com/dyn4j/dyn4j-samples/blob/11c2241bd45fd28f2a8012ef9675191b0a67c847/src/main/java/org/dyn4j/samples/BasketBall.java#L360
 * 
 * if you want to remove an item you need to keep a list of items to remove and
 * then remove them in steplistener.end.
 * 
 * https://dyn4j.org/pages/advanced.html#Collision_Detection
 * 
 * so in our case we'd want an "amp" sensor and a Filter that allows the note to
 * pass over the side walls (i.e. looking at the altitude) but to hit the amp sensor.
 * 
 */
public class Scorekeeper {

}
