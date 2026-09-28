package org.sih.seamlessnav

import android.location.Location
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin
import kotlin.math.sqrt

/** Phone-side 10 Hz navigation state. IMU propagation runs at sensor rate. */
data class PhoneNavigationState(
    val timestampNs: Long,
    val latitudeDeg: Double,
    val longitudeDeg: Double,
    val speedMps: Double,
    val headingDeg: Double,
    val horizontalSigmaM: Double,
    val mode: String,
    val acceptedGnss: Boolean
)

/**
 * Compact planar error-state filter. Inputs are gravity-compensated forward
 * acceleration and vehicle-up yaw rate in SI units. GNSS updates are quality
 * weighted; signal-quality gates control inertial process noise and NHC/ZUPT.
 * Learned speed is intentionally not an input to this class.
 */
class PhoneNavigationEngine(private val outputHz: Double = 10.0) {
    private val x = DoubleArray(7) // E, N, vE, vN, heading (E/CCW), gyro bias, accel bias
    private val p = Array(7) { DoubleArray(7) }
    private val outputPeriodNs = (1e9 / outputHz.coerceIn(1.0, 50.0)).toLong().coerceAtLeast(1L)
    private var lastImuNs: Long? = null
    private var nextOutputNs: Long? = null
    private var lastGnssNs: Long? = null
    private var originLat: Double? = null
    private var originLon: Double? = null
    private var latest: PhoneNavigationState? = null
    private var acceptedGnss = false

    init {
        val variances = doubleArrayOf(100.0, 100.0, 9.0, 9.0, (PI / 6) * (PI / 6), (PI / 90) * (PI / 90), 1.0)
        for (i in 0..6) p[i][i] = variances[i]
    }

    @Synchronized
    fun updateGnss(location: Location): PhoneNavigationState? {
        if (!location.latitude.isFinite() || !location.longitude.isFinite() || !location.accuracy.isFinite()) return latest
        if (originLat == null) {
            originLat = location.latitude
            originLon = location.longitude
            x[0] = 0.0; x[1] = 0.0
            p[0][0] = max(1.0, location.accuracy.toDouble().pow2())
            p[1][1] = p[0][0]
            if (location.hasSpeed() && location.speed >= 1f && location.hasBearing()) {
                x[4] = courseToHeading(location.bearing.toDouble())
                x[2] = location.speed * cos(x[4]); x[3] = location.speed * sin(x[4])
            }
            lastGnssNs = location.elapsedRealtimeNanos
            acceptedGnss = true
            latest = makeState(location.elapsedRealtimeNanos)
            return latest
        }
        val lat0 = originLat!!
        val lon0 = originLon!!
        val east = Math.toRadians(location.longitude - lon0) * EARTH_RADIUS_M * cos(Math.toRadians(lat0))
        val north = Math.toRadians(location.latitude - lat0) * EARTH_RADIUS_M
        val sigma = location.accuracy.toDouble().coerceIn(1.0, 100.0)
        val priorEast = x[0]; val priorNorth = x[1]
        val posD2 = (east - priorEast).pow2() / (p[0][0] + sigma * sigma) + (north - priorNorth).pow2() / (p[1][1] + sigma * sigma)
        acceptedGnss = posD2 <= 25.0
        if (acceptedGnss) {
            scalarUpdate(0, east - x[0], max(1.0, sigma * sigma), 25.0)
            scalarUpdate(1, north - x[1], max(1.0, sigma * sigma), 25.0)
            if (location.hasSpeed() && location.speed.isFinite() && location.speed >= 0f) {
                if (location.hasBearing() && location.speed >= 1f) {
                    val heading = courseToHeading(location.bearing.toDouble())
                    val ve = location.speed * cos(heading)
                    val vn = location.speed * sin(heading)
                    val speedSigma = 0.6 + sigma * 0.03
                    scalarUpdate(2, ve - x[2], speedSigma.pow2(), 30.0)
                    scalarUpdate(3, vn - x[3], speedSigma.pow2(), 30.0)
                    scalarUpdate(4, wrap(heading - x[4]), Math.toRadians(12.0).pow2(), 20.0)
                } else {
                    val h = DoubleArray(7)
                    h[2] = cos(x[4]); h[3] = sin(x[4])
                    val predicted = x[2] * h[2] + x[3] * h[3]
                    scalarUpdate(h, location.speed - predicted, 1.0, 16.0)
                }
            }
            lastGnssNs = location.elapsedRealtimeNanos
        }
        latest = makeState(max(location.elapsedRealtimeNanos, lastImuNs ?: 0L))
        return latest
    }

    @Synchronized
    fun processImu(
        timestampNs: Long,
        forwardAccelerationMps2: Double,
        yawRateRadps: Double,
        assessment: SignalAssessment
    ): PhoneNavigationState? {
        if (timestampNs <= 0 || !forwardAccelerationMps2.isFinite() || !yawRateRadps.isFinite()) return latest
        val previous = lastImuNs
        if (previous != null && timestampNs <= previous) return latest
        val dt = if (previous == null) 0.0 else (timestampNs - previous) / 1e9
        lastImuNs = timestampNs
        if (nextOutputNs == null) nextOutputNs = timestampNs
        if (dt > 0.0) {
            val steps = max(1, kotlin.math.ceil(dt / 0.05).toInt())
            val q = assessment.quality.coerceIn(0.0, 1.0)
            repeat(steps) { predict(dt / steps, forwardAccelerationMps2, yawRateRadps, q) }
        }
        if (assessment.stationary) {
            scalarUpdate(2, -x[2], 0.04, null)
            scalarUpdate(3, -x[3], 0.04, null)
            if (assessment.quality >= 0.5) {
                scalarUpdate(5, yawRateRadps - x[5], 0.04, 16.0)
                scalarUpdate(6, forwardAccelerationMps2 - x[6], 0.08, 16.0)
            }
        } else if (assessment.normalDriving && assessment.quality >= 0.35 && hypot(x[2], x[3]) > 1.0) {
            val h = DoubleArray(7)
            h[2] = -sin(x[4]); h[3] = cos(x[4])
            h[4] = -cos(x[4]) * x[2] - sin(x[4]) * x[3]
            val lateral = -sin(x[4]) * x[2] + cos(x[4]) * x[3]
            val sigma = 0.15 + (1.0 - assessment.quality) * 2.0
            scalarUpdate(h, -lateral, sigma * sigma, null)
        }
        val due = nextOutputNs ?: timestampNs
        if (timestampNs < due) return latest
        nextOutputNs = due + ((timestampNs - due) / outputPeriodNs + 1) * outputPeriodNs
        latest = makeState(timestampNs)
        return latest
    }

    @Synchronized fun snapshot(): PhoneNavigationState? = latest

    @Synchronized
    fun reset() {
        x.fill(0.0)
        for (row in p) row.fill(0.0)
        val vars = doubleArrayOf(100.0, 100.0, 9.0, 9.0, (PI / 6) * (PI / 6), (PI / 90) * (PI / 90), 1.0)
        for (i in 0..6) p[i][i] = vars[i]
        lastImuNs = null; nextOutputNs = null; lastGnssNs = null
        originLat = null; originLon = null; latest = null; acceptedGnss = false
    }

    private fun predict(dt: Double, accelForward: Double, yawRate: Double, quality: Double) {
        val yaw = x[4]
        val c = cos(yaw); val s = sin(yaw)
        val accel = accelForward - x[6]
        x[0] += x[2] * dt + 0.5 * accel * c * dt * dt
        x[1] += x[3] * dt + 0.5 * accel * s * dt * dt
        x[2] += accel * c * dt
        x[3] += accel * s * dt
        x[4] = wrap(yaw + (yawRate - x[5]) * dt)
        val f = Array(7) { i -> DoubleArray(7) { j -> if (i == j) 1.0 else 0.0 } }
        f[0][2] = dt; f[1][3] = dt
        f[0][4] = -0.5 * accel * s * dt * dt; f[1][4] = 0.5 * accel * c * dt * dt
        f[2][4] = -accel * s * dt; f[3][4] = accel * c * dt
        f[0][6] = -0.5 * c * dt * dt; f[1][6] = -0.5 * s * dt * dt
        f[2][6] = -c * dt; f[3][6] = -s * dt; f[4][5] = -dt
        val fp = multiply(f, p)
        val nextP = multiply(fp, transpose(f))
        val accelSigma = 0.35 + (1.0 - quality) * 4.0
        val gyroSigma = Math.toRadians(0.5 + (1.0 - quality) * 8.0)
        val q = doubleArrayOf(
            (0.5 * accelSigma * dt * dt).pow2(), (0.5 * accelSigma * dt * dt).pow2(),
            (accelSigma * dt).pow2(), (accelSigma * dt).pow2(), (gyroSigma * dt).pow2(),
            (Math.toRadians(0.02) * sqrt(dt)).pow2(), (0.01 * sqrt(dt)).pow2()
        )
        for (i in 0..6) nextP[i][i] += q[i]
        for (i in 0..6) for (j in 0..6) p[i][j] = 0.5 * (nextP[i][j] + nextP[j][i])
    }

    private fun scalarUpdate(index: Int, residual: Double, variance: Double, gate: Double?) {
        val h = DoubleArray(7); h[index] = 1.0
        scalarUpdate(h, residual, variance, gate)
    }

    private fun scalarUpdate(h: DoubleArray, residual: Double, variance: Double, gate: Double?): Boolean {
        if (!residual.isFinite() || !variance.isFinite() || variance <= 0.0) return false
        val ph = DoubleArray(7) { i -> (0..6).sumOf { j -> p[i][j] * h[j] } }
        val innovation = variance + (0..6).sumOf { i -> h[i] * ph[i] }
        if (innovation <= 0.0 || !innovation.isFinite() || gate != null && residual * residual / innovation > gate) return false
        val k = DoubleArray(7) { ph[it] / innovation }
        for (i in 0..6) x[i] += k[i] * residual
        x[4] = wrap(x[4])
        val old = Array(7) { p[it].copyOf() }
        // Joseph form for one scalar observation: (I-KH)P(I-KH)' + KRK'.
        val a = Array(7) { i -> DoubleArray(7) { j -> (if (i == j) 1.0 else 0.0) - k[i] * h[j] } }
        val joseph = multiply(multiply(a, old), transpose(a))
        for (i in 0..6) for (j in 0..6) p[i][j] = joseph[i][j] + k[i] * variance * k[j]
        return true
    }

    private fun makeState(timestampNs: Long): PhoneNavigationState? {
        val lat0 = originLat ?: return null
        val lon0 = originLon ?: return null
        val latitude = lat0 + Math.toDegrees(x[1] / EARTH_RADIUS_M)
        val longitude = lon0 + Math.toDegrees(x[0] / (EARTH_RADIUS_M * cos(Math.toRadians(lat0)).coerceAtLeast(1e-6)))
        val gnssAge = lastGnssNs?.let { max(0L, timestampNs - it) / 1e9 } ?: Double.POSITIVE_INFINITY
        val headingBearing = ((90.0 - Math.toDegrees(x[4]) % 360.0) + 360.0) % 360.0
        return PhoneNavigationState(
            timestampNs, latitude, longitude, hypot(x[2], x[3]), headingBearing,
            sqrt(max(0.0, p[0][0] + p[1][1])), if (gnssAge <= 1.5) "GNSS_AIDED" else "DEAD_RECKONING", acceptedGnss
        )
    }

    private fun courseToHeading(courseDeg: Double) = wrap(PI / 2.0 - Math.toRadians(courseDeg))
    private fun wrap(angle: Double): Double = (angle + PI) % (2.0 * PI) - PI
    private fun Double.pow2() = this * this

    private fun multiply(a: Array<DoubleArray>, b: Array<DoubleArray>): Array<DoubleArray> =
        Array(a.size) { i -> DoubleArray(b[0].size) { j -> (b.indices).sumOf { k -> a[i][k] * b[k][j] } } }
    private fun transpose(a: Array<DoubleArray>) = Array(a[0].size) { j -> DoubleArray(a.size) { i -> a[i][j] } }

    companion object { private const val EARTH_RADIUS_M = 6_378_137.0 }
}
