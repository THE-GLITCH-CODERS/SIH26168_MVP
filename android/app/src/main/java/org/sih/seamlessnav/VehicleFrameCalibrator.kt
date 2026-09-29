package org.sih.seamlessnav

import kotlin.math.*

/** Learns a right-handed x-forward/y-left/z-up mount rotation from gravity and
 * signed acceleration on straight GNSS-aided segments. Requires near-level road.
 * Quiet constant-speed driving is unobservable and never completes calibration. */
class VehicleFrameCalibrator {
    data class Result(val ready: Boolean, val confidence: Double, val observations: Int,
        val rotation: DoubleArray?, val rollDeg: Double?, val pitchDeg: Double?, val yawDeg: Double?, val reason: String)
    private data class Sample(val time: Long, val linear: DoubleArray, val gravity: DoubleArray)
    private val samples = ArrayDeque<Sample>()
    private val directions = ArrayDeque<DoubleArray>()
    private var lastFix: Triple<Long, Double, Double>? = null
    private var up = doubleArrayOf(0.0,0.0,1.0)
    private var lockedUp: DoubleArray? = null
    private var shiftSamples = 0
    private var latest = Result(false,0.0,0,null,null,null,null,"Waiting for straight acceleration with good GNSS")

    @Synchronized fun observeImu(time: Long, linear: DoubleArray, gravity: DoubleArray, mountShift: Boolean) {
        if (mountShift) { reset(); latest = latest.copy(reason="Mount moved; recollecting alignment"); return }
        if (linear.size != 3 || gravity.size != 3 || (linear+gravity).any { !it.isFinite() } || norm(gravity) !in 7.0..12.0) return
        if (samples.lastOrNull()?.time?.let { time <= it } == true) return
        val g = unit(gravity)
        shiftSamples = if (lockedUp?.let { dot(it,g) < cos(Math.toRadians(15.0)) } == true) shiftSamples+1 else 0
        if (shiftSamples >= 20) { reset(); latest = latest.copy(reason="Gravity direction changed; recalibration required") }
        samples.addLast(Sample(time,linear.copyOf(),gravity.copyOf()))
        while(samples.isNotEmpty() && time-samples.first().time > 3_000_000_000L) samples.removeFirst()
    }

    @Synchronized fun observeGnss(time: Long, speed: Double, course: Double, accuracy: Double) {
        if (!speed.isFinite() || !course.isFinite() || !accuracy.isFinite() || accuracy !in 0.01..15.0 || speed < 2.0) { lastFix=null; return }
        val previous=lastFix
        if(previous!=null && time<=previous.first) return
        lastFix=Triple(time,speed,course)
        if(previous==null) return
        val dt=(time-previous.first)/1e9
        val turn=abs(((course-previous.third+540)%360)-180)
        if(dt !in 0.5..2.0 || turn>5.0) return
        val a=(speed-previous.second)/dt
        if(abs(a) !in 0.3..4.0) return
        val interval=samples.filter { it.time in previous.first..time }
        if(interval.size<4 || interval.last().time-interval.first().time < (0.6*dt*1e9).toLong()) return
        val mean=DoubleArray(3) { k->interval.sumOf { it.linear[k] }/interval.size }
        up=unit(DoubleArray(3) { k->interval.sumOf { it.gravity[k] }/interval.size })
        val horizontal=DoubleArray(3) { k->mean[k]-dot(mean,up)*up[k] }
        if(norm(horizontal) !in 0.2..5.0) return
        val direction=unit(DoubleArray(3) { horizontal[it]*sign(a) })
        directions.addLast(direction)
        while(directions.size>30) directions.removeFirst()
        val average=DoubleArray(3) { k->directions.sumOf { it[k] }/directions.size }
        val coherence=norm(average).coerceIn(0.0,1.0)
        val forward=unit(DoubleArray(3) { average[it]-dot(average,up)*up[it] })
        val left=unit(cross(up,forward))
        val correctedUp=unit(cross(forward,left))
        val rotation=forward+left+correctedUp
        val confidence=coherence*min(1.0,directions.size/10.0)
        val ready=confidence>=0.75 && coherence>=0.93
        if(ready) lockedUp=correctedUp.copyOf()
        latest=Result(ready,confidence,directions.size,if(ready) rotation else null,
            if(ready) Math.toDegrees(atan2(rotation[7],rotation[8])) else null,
            if(ready) Math.toDegrees(asin((-rotation[6]).coerceIn(-1.0,1.0))) else null,
            if(ready) Math.toDegrees(atan2(rotation[3],rotation[0])) else null,
            if(ready) "Gravity + signed straight-motion alignment; level-road assumption" else "Collecting observable acceleration/braking segments")
    }
    @Synchronized fun snapshot()=latest.copy(rotation=latest.rotation?.copyOf())
    @Synchronized fun project(linear: DoubleArray, gyro: DoubleArray): Pair<Double,Double>? {
        val r=latest.rotation ?: return null
        return (0..2).sumOf { r[it]*linear[it] } to (0..2).sumOf { r[6+it]*gyro[it] }
    }
    @Synchronized fun reset() { samples.clear(); directions.clear(); lastFix=null; lockedUp=null; shiftSamples=0; latest=Result(false,0.0,0,null,null,null,null,"Waiting for straight acceleration with good GNSS") }
    private fun norm(v: DoubleArray)=sqrt(v.sumOf { it*it })
    private fun unit(v: DoubleArray): DoubleArray { val n=norm(v).coerceAtLeast(1e-9); return DoubleArray(3) { v[it]/n } }
    private fun dot(a: DoubleArray,b: DoubleArray)=(0..2).sumOf { a[it]*b[it] }
    private fun cross(a: DoubleArray,b: DoubleArray)=doubleArrayOf(a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])
}
