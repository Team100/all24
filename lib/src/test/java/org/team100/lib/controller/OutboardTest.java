package org.team100.lib.controller;

/**
 * Our critical axes are driven with outboard closed-loop controllers, either
 * position (for things like steering or arm location) or velocity (for things
 * like driving or rolly grabbers).
 * 
 * We don't use any outboard profiling features, we just use PID control.
 * 
 * We also supply "arbitrary feed forward" voltage, typically calculated using a
 * motor model that includes static friction, dynamic friction, back EMF, and
 * acceleration. We also include "extra torque" in the feedforward, typically
 * used for gravity compensation for velocity control.
 * 
 * How important are all these terms? If the outboard PID controller is firm
 * enough, can it do all the work?  What kA and P would work together?
 * 
 * 
 */
public class OutboardTest {

}
