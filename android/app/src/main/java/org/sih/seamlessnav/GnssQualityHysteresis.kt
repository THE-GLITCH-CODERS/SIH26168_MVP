package org.sih.seamlessnav

/** Latches degraded GNSS until several consecutive high-quality fixes recover. */
internal class GnssQualityHysteresis(
    private val enterDegradedAccuracyM: Double = 20.0,
    private val exitDegradedAccuracyM: Double = 12.0,
    private val requiredGoodFixes: Int = 3
) {
    var degraded: Boolean = false
        private set
    private var consecutiveGoodFixes = 0

    fun onAcceptedFix(accuracyM: Double) {
        if (!accuracyM.isFinite() || accuracyM >= enterDegradedAccuracyM) {
            markDegraded()
            return
        }
        if (accuracyM > exitDegradedAccuracyM) {
            consecutiveGoodFixes = 0
            return
        }
        if (!degraded) return
        consecutiveGoodFixes++
        if (consecutiveGoodFixes >= requiredGoodFixes) {
            degraded = false
            consecutiveGoodFixes = 0
        }
    }

    fun markDegraded() {
        degraded = true
        consecutiveGoodFixes = 0
    }

    fun reset() {
        degraded = false
        consecutiveGoodFixes = 0
    }
}
