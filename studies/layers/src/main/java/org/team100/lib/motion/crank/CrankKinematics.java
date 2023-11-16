package org.team100.lib.motion.crank;

/**
 * Kinematics of a rotating crank, like a steam locomotive.
 * 
 * The state can be expressed as the linear slider position/velocity
 * or as the crank rotation/angular velocity.
 * 
 * The crank is the actuated thing, so the transformation from crank to slider
 * is the "forward" transform and the transformation from slider to crank is the
 * "inverse" transform.
 * 
 * This reference defines x as displacement from top dead center:
 * https://www.alistairstutorials.co.uk/tutorial13.html
 * 
 * This reference defines x as distance from slider to crank axis,
 * which is what we use here:
 * https://en.wikipedia.org/wiki/Slider-crank_linkage
 * https://en.wikipedia.org/wiki/Piston_motion_equations
 */
public class CrankKinematics {
    private final double m_crankRadius;
    private final double m_rodLength;

    /** Length units can be anything, they just need to be consistent. */
    public CrankKinematics(double crankRadius, double rodLength) {
        m_crankRadius = crankRadius;
        m_rodLength = rodLength;
    }

    /**
     * Given a point in configuration space (crank angle), returns a point in work
     * space (slider position).
     * 
     * @return slider position measured from the crank axis, using the same units
     *         used in the constructor.
     */
    public CrankWorkstate forward(CrankConfiguration crankAngleRad) {
        // this is directly from wikipedia
        double cosAngle = Math.cos(crankAngleRad.getCrankAngleRad());
        double sinAngle = Math.sin(crankAngleRad.getCrankAngleRad());
        return new CrankWorkstate(m_crankRadius * cosAngle + Math.sqrt(m_rodLength * m_rodLength
                - m_crankRadius * m_crankRadius * sinAngle * sinAngle));
    }

    /**
     * Given a point in work space (slider position), returns a point in
     * configuration space (crank angle).
     * 
     * @param sliderPosition measured from the crank axis, using the same units used
     *                       in the constructor.
     */
    public CrankConfiguration inverse(CrankWorkstate sliderPosition) {
        // this is the wikipedia expression inverted.
        return new CrankConfiguration(
                Math.acos((sliderPosition.getState() * sliderPosition.getState() + m_crankRadius * m_crankRadius
                        - m_rodLength * m_rodLength) / (2 * sliderPosition.getState() * m_crankRadius)));
    }
}
