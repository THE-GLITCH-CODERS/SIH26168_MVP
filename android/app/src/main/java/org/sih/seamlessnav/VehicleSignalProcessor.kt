package org.sih.seamlessnav

import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

/** Sensor-neutral, SI-unit input contract for phone and external-IMU adapters. */
data class ImuVectorSample(
    val timestampNs: Long,
    val accelerationMps2: DoubleArray? = null,
    val gravityMps2: DoubleArray? = null,
    val linearAccelerationMps2: DoubleArray? = null,
    val angularRateRadps: DoubleArray? = null,
    /** Device azimuth clockwise from magnetic north, radians, when available. */
    val deviceAzimuthRad: Double? = null
)

data class SignalAssessment(
    val event: String,
    val quality: Double,
    val stationary: Boolean,
    val normalDriving: Boolean,
    val alignmentConfidence: Double,
    val alignmentOffsetDeg: Double?,
    val phonePitchDeg: Double?,
    val phoneRollDeg: Double?,
    val gyroBiasRadps: DoubleArray,
    val calibrated: Boolean,
    val mountShiftSuspected: Boolean,
    val sampleRateHz: Double?,
    val detail: String
)

/**
 * First live signal-quality/calibration block from the architecture.
 * It is deliberately conservative: no position or speed is synthesized here.
 */
class VehicleSignalProcessor {
    private val gyroBias = DoubleArray(3)
    private var biasSamples = 0L
    private var stationarySeconds = 0.0
    private var previousTimestampNs: Long? = null
    private var previousLinearNorm: Double? = null
    private var accelCount = 0L
    private var firstAccelNs: Long? = null
    private val linearNormWindow = ArrayDeque<Double>()

    private var acceleration = DoubleArray(3)
    private var gravity = DoubleArray(3)
    private var linearAcceleration = DoubleArray(3)
    private var angularRate = DoubleArray(3)
    private var hasGravity = false
    private var hasLinearAcceleration = false
    private var azimuthRad: Double? = null
    private var pitchRad: Double? = null
    private var rollRad: Double? = null
    private var orientationTimestampNs: Long? = null
    private var gnssSpeedMps: Double? = null
    private var gnssBearingRad: Double? = null
    private var gnssAccuracyM: Double? = null
    private var gnssTimestampNs: Long? = null
    private var lastKnownMoving = false
    private var alignmentOffsetRad: Double? = null
    private val alignmentResiduals = ArrayDeque<Double>()
    private var mountShiftSuspected = false
    private var lastAssessment = SignalAssessment(
        event = "WAITING_FOR_IMU",
        quality = 0.0,
        stationary = false,
        normalDriving = false,
        alignmentConfidence = 0.0,
        alignmentOffsetDeg = null,
        phonePitchDeg = null,
        phoneRollDeg = null,
        gyroBiasRadps = gyroBias.copyOf(),
        calibrated = false,
        mountShiftSuspected = false,
        sampleRateHz = null,
        detail = "Start capture to assess motion and alignment."
    )

    @Synchronized
    fun updateSensor(
        name: String,
        timestampNs: Long,
        values: DoubleArray,
        deviceAzimuthRad: Double? = null,
        devicePitchRad: Double? = null,
        deviceRollRad: Double? = null
    ): SignalAssessment {
        if (timestampNs <= 0L || values.size < 3 || values.take(3).any { !it.isFinite() }) return lastAssessment
        when (name) {
            "accelerometer" -> acceleration = values.copyOfRange(0, 3)
            "gravity" -> { gravity = values.copyOfRange(0, 3); hasGravity = true }
            "linear_acceleration" -> { linearAcceleration = values.copyOfRange(0, 3); hasLinearAcceleration = true }
            "gyroscope" -> angularRate = values.copyOfRange(0, 3)
            "rotation_vector" -> if (deviceAzimuthRad?.isFinite() == true) azimuthRad = deviceAzimuthRad
        }
        if (deviceAzimuthRad?.isFinite() == true) {
            azimuthRad = deviceAzimuthRad
            if (devicePitchRad?.isFinite() == true) pitchRad = devicePitchRad
            if (deviceRollRad?.isFinite() == true) rollRad = deviceRollRad
            orientationTimestampNs = timestampNs
        }
        if (name != "accelerometer") return lastAssessment

        val dt = previousTimestampNs?.let { (timestampNs - it) / 1e9 } ?: 0.0
        if (dt < 0.0) return lastAssessment
        previousTimestampNs = timestampNs
        if (firstAccelNs == null) firstAccelNs = timestampNs
        accelCount++
        val rate = firstAccelNs?.let { start ->
            if (timestampNs > start && accelCount > 1) (accelCount - 1) * 1e9 / (timestampNs - start) else null
        }

        val linear = when {
            hasLinearAcceleration -> linearAcceleration
            hasGravity -> DoubleArray(3) { acceleration[it] - gravity[it] }
            else -> DoubleArray(3)
        }
        val linearNorm = norm(linear)
        val gyroNorm = norm(DoubleArray(3) { angularRate[it] - gyroBias[it] })
        val verticalGyro = if (norm(gravity) > 1.0) dot(angularRate, gravity) / norm(gravity) else angularRate[2]
        val verticalGyroBias = if (norm(gravity) > 1.0) dot(gyroBias, gravity) / norm(gravity) else gyroBias[2]
        val jerk = if (dt > 1e-4 && dt < 0.25 && previousLinearNorm != null) abs(linearNorm - previousLinearNorm!!) / dt else 0.0
        previousLinearNorm = linearNorm
        linearNormWindow.addLast(linearNorm)
        while (linearNormWindow.size > 50) linearNormWindow.removeFirst()
        val vibrationRms = if (linearNormWindow.size >= 10) {
            val mean = linearNormWindow.average()
            sqrt(linearNormWindow.map { (it - mean) * (it - mean) }.average())
        } else 0.0

        val gnssAgeSeconds = gnssTimestampNs?.let { max(0.0, (timestampNs - it) / 1e9) } ?: Double.POSITIVE_INFINITY
        val gnssFresh = gnssAgeSeconds <= 2.0 && (gnssAccuracyM ?: Double.POSITIVE_INFINITY) <= 30.0
        val speed = gnssSpeedMps
        val stationaryEvidence = if (gnssFresh && speed != null) {
            speed < 0.8 && gyroNorm < 0.10 && linearNorm < 0.55
        } else {
            // Constant-speed travel can have the same quiet IMU signal as rest.
            // Never zero velocity merely because a moving car enters a smooth tunnel.
            !lastKnownMoving && gyroNorm < 0.045 && linearNorm < 0.22
        }
        stationarySeconds = if (stationaryEvidence && dt in 0.001..0.2) stationarySeconds + dt else if (stationaryEvidence) stationarySeconds else 0.0
        if (stationarySeconds >= 2.0 && gyroNorm < 0.10 && linearNorm < 0.55) {
            // Slow, bounded mean update prevents one noisy stationary sample from corrupting bias.
            biasSamples++
            val gain = min(1.0 / biasSamples.toDouble(), 0.02)
            for (axis in 0..2) gyroBias[axis] += gain * (angularRate[axis] - gyroBias[axis])
        }

        val impact = linearNorm > 5.0 || jerk > 55.0
        val turning = abs(verticalGyro - verticalGyroBias) > 0.14
        val highVibration = !impact && vibrationRms > 0.85
        val stationary = stationarySeconds >= 2.0
        if (gnssFresh && speed != null) lastKnownMoving = speed > 2.0
        // Carry a recent GNSS motion state through an outage. Clear it only
        // after sustained IMU stationary evidence so NHC remains available
        // during a smooth, GNSS-denied drive.
        val moving = if (gnssFresh && speed != null) speed > 2.0 else lastKnownMoving && stationarySeconds < 2.0
        val event = when {
            mountShiftSuspected -> "MOUNT_SHIFT_SUSPECTED"
            impact -> "IMPACT_OR_POTHOLE"
            stationary -> "STATIONARY"
            highVibration -> "HIGH_VIBRATION"
            turning -> "TURNING"
            moving && alignmentOffsetRad != null -> "ALIGNED_DRIVING"
            moving -> "MOVING_CALIBRATION"
            else -> "MOTION_UNCERTAIN"
        }
        var quality = when {
            impact -> 0.15
            highVibration -> 0.35
            dt > 0.1 -> 0.45
            else -> 0.75
        }
        if (mountShiftSuspected) quality = min(quality, 0.20)
        else if (alignmentOffsetRad != null) quality = min(1.0, quality + 0.15 * alignmentConfidence())
        val normalDriving = moving && !impact && !highVibration && !mountShiftSuspected
        val calibrated = biasSamples > 0L && alignmentOffsetRad != null && alignmentConfidence() >= 0.45 && !mountShiftSuspected
        val detail = when {
            mountShiftSuspected -> "Orientation/course residual changed; recalibration is needed."
            impact -> "Large acceleration transient; vehicle constraints should be relaxed."
            highVibration -> "Elevated short-window vibration; inertial confidence reduced."
            stationary && alignmentOffsetRad == null -> "Stationary gyro-bias estimate ready; await a straight GNSS-aided segment for yaw alignment."
            alignmentOffsetRad == null -> "Yaw alignment needs a fresh GNSS course while moving above 2 m/s."
            calibrated -> "Stationary bias and GNSS-aided vehicle alignment are available."
            else -> "Collecting stable motion and GNSS course observations."
        }
        lastAssessment = SignalAssessment(
            event = event,
            quality = quality,
            stationary = stationary,
            normalDriving = normalDriving,
            alignmentConfidence = alignmentConfidence(),
            alignmentOffsetDeg = alignmentOffsetRad?.let { Math.toDegrees(it) },
            phonePitchDeg = pitchRad?.let { Math.toDegrees(it) },
            phoneRollDeg = rollRad?.let { Math.toDegrees(it) },
            gyroBiasRadps = gyroBias.copyOf(),
            calibrated = calibrated,
            mountShiftSuspected = mountShiftSuspected,
            sampleRateHz = rate,
            detail = detail
        )
        return lastAssessment
    }

    @Synchronized
    fun updateGnss(timestampNs: Long, speedMps: Double?, bearingDeg: Double?, accuracyM: Double?) {
        if (timestampNs <= 0L) return
        gnssTimestampNs = timestampNs
        gnssSpeedMps = speedMps?.takeIf { it.isFinite() && it >= 0.0 }
        gnssBearingRad = bearingDeg?.takeIf { it.isFinite() }?.let { Math.toRadians(it) }
        gnssAccuracyM = accuracyM?.takeIf { it.isFinite() && it > 0.0 }
        updateAlignment(timestampNs)
    }

    @Synchronized
    fun snapshot(): SignalAssessment = lastAssessment.copy(gyroBiasRadps = gyroBias.copyOf())

    @Synchronized
    fun reset() {
        gyroBias.fill(0.0)
        biasSamples = 0L
        stationarySeconds = 0.0
        previousTimestampNs = null
        previousLinearNorm = null
        accelCount = 0L
        firstAccelNs = null
        acceleration.fill(0.0)
        gravity.fill(0.0)
        linearAcceleration.fill(0.0)
        angularRate.fill(0.0)
        hasGravity = false
        hasLinearAcceleration = false
        linearNormWindow.clear()
        alignmentOffsetRad = null
        orientationTimestampNs = null
        alignmentResiduals.clear()
        mountShiftSuspected = false
        gnssSpeedMps = null
        gnssBearingRad = null
        gnssAccuracyM = null
        gnssTimestampNs = null
        lastKnownMoving = false
        lastAssessment = lastAssessment.copy(
            event = "WAITING_FOR_IMU", quality = 0.0, stationary = false, normalDriving = false,
            alignmentConfidence = 0.0, alignmentOffsetDeg = null, gyroBiasRadps = gyroBias.copyOf(),
            phonePitchDeg = null, phoneRollDeg = null,
            calibrated = false, mountShiftSuspected = false, sampleRateHz = null,
            detail = "Start capture to assess motion and alignment."
        )
    }

    private fun updateAlignment(nowNs: Long) {
        val azimuth = azimuthRad ?: return
        val orientationTimestamp = orientationTimestampNs ?: return
        val bearing = gnssBearingRad ?: return
        val speed = gnssSpeedMps ?: return
        val timestamp = gnssTimestampNs ?: return
        if (nowNs - timestamp !in 0..2_000_000_000L || nowNs - orientationTimestamp !in 0..1_000_000_000L || speed < 4.0 || (gnssAccuracyM ?: 999.0) > 20.0) return
        val observedOffset = wrap(bearing - azimuth)
        val old = alignmentOffsetRad
        if (old != null && !mountShiftSuspected && abs(wrap(observedOffset - old)) > Math.toRadians(32.0) && alignmentResiduals.size >= 8) {
            mountShiftSuspected = true
            alignmentResiduals.clear()
        }
        alignmentResiduals.addLast(observedOffset)
        while (alignmentResiduals.size > 20) alignmentResiduals.removeFirst()
        val meanSin = alignmentResiduals.sumOf { kotlin.math.sin(it) }
        val meanCos = alignmentResiduals.sumOf { cos(it) }
        val next = atan2(meanSin, meanCos)
        if (old == null || !mountShiftSuspected) alignmentOffsetRad = next
        if (mountShiftSuspected && alignmentResiduals.size >= 12) {
            val spread = alignmentResiduals.maxOf { abs(wrap(it - next)) }
            if (spread < Math.toRadians(12.0)) {
                alignmentOffsetRad = next
                mountShiftSuspected = false
            }
        }
    }

    private fun alignmentConfidence(): Double {
        if (alignmentOffsetRad == null || alignmentResiduals.size < 3) return 0.0
        val meanSin = alignmentResiduals.sumOf { kotlin.math.sin(it) } / alignmentResiduals.size
        val meanCos = alignmentResiduals.sumOf { cos(it) } / alignmentResiduals.size
        val concentration = hypot(meanSin, meanCos).coerceIn(0.0, 1.0)
        val sampleCoverage = min(1.0, alignmentResiduals.size / 12.0)
        return concentration * sampleCoverage
    }

    private fun norm(vector: DoubleArray): Double = sqrt(vector.sumOf { it * it })
    private fun dot(a: DoubleArray, b: DoubleArray): Double = a.indices.sumOf { a[it] * b[it] }
    private fun wrap(angle: Double): Double = (angle + Math.PI) % (2.0 * Math.PI) - Math.PI
}
