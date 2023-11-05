package org.team100.controllib.simulation;

public class CompleteState {
    private static final double kSecPerUsec = 1e-6;
    // int step;
    long systemTimeMicrosec; // from FPGAtime

    double actualTimeSec() {
        return systemTimeMicrosec * kSecPerUsec;
    }

    double actualPosition;
    double actualVelocity;
    double actualAcceleration;

    double positionObservationTimeSec; // valid time this observation represents
    double observedPosition;
    double velocityObservationTimeSec; // valid time this observation represents
    double observedVelocity;
    double observedAcceleration; // for these tests this is intended to match u.

    int replayCount;

    double predictionTimeSec; // time in the future this prediction is intended for
    double predictedPosition;
    double predictedVelocity;

    double referencePosition;
    double referenceVelocity;
    double referenceAcceleration;

    double residualPosition;
    double residualVelocity;

    double ffU;
    double errorPosition;
    double errorVelocity;
    double fbU;
    double totalU;

    public void init(
            double initialPosition,
            double initialVelocity,
            double initialAcceleration) {
        systemTimeMicrosec = 0l;

        actualPosition = initialPosition;
        actualVelocity = initialVelocity;
        actualAcceleration = initialAcceleration;

        observedPosition = initialPosition;
        observedVelocity = initialVelocity;
        observedAcceleration = initialAcceleration;

        predictedPosition = initialPosition;
        predictedVelocity = initialVelocity;

        referencePosition = 0;
        referenceVelocity = 0;
        referenceAcceleration = 0;

        residualPosition = 0;
        residualVelocity = 0;

        ffU = 0;
        errorPosition = 0;
        errorVelocity = 0;
        fbU = 0;
        totalU = initialAcceleration;
    }


    public String header() {
        return "     sysTime,   actualTime, " +
                "   actualPos,    actualVel,    actualAcc, " +
                " observedPos,  observedVel,  observedAcc, replayCount, " +
                "predictedPos, predictedVel, " +
                "referencePos, referenceVel, referenceAcc, " +
                " residualPos,  residualVel, " +
                "         ffU,     errorPos,     errorVel,          fbU,       totalU";
    }

    public String toString() {
        String format = "%12d, %12.3f, " +
                "%12.3f, %12.3f, %12.3f, " +
                "%12.3f, %12.3f, %12.3f, %12d, " +
                "%12.3f, %12.3f, " +
                "%12.3f, %12.3f, %12.3f, " +
                "%12.3f, %12.3f, " +
                "%12.3f, %12.5f, %12.5f, %12.3f, %12.3f";
        return String.format(format,
                systemTimeMicrosec,
                actualTimeSec(),
                actualPosition,
                actualVelocity,
                actualAcceleration,
                observedPosition,
                observedVelocity,
                observedAcceleration,
                replayCount,
                predictedPosition,
                predictedVelocity,
                referencePosition,
                referenceVelocity,
                referenceAcceleration,
                residualPosition,
                residualVelocity,
                ffU,
                errorPosition,
                errorVelocity,
                fbU,
                totalU);

    }
}
