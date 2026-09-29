package org.sih.seamlessnav

import org.json.JSONObject
import kotlin.math.*

/** Portable learned speed/variance inference. High-rate inputs are averaged into
 * completed 100 ms magnitude bins; windows never cross gaps or use future bins. */
class PortableSpeedModel(private val document: JSONObject) {
    data class Prediction(val timestampNs: Long, val speedMps: Double, val variance: Double,
        val outOfDomain: Boolean, val eligibleForFusion: Boolean, val vibrationRms: Double)
    private fun vector(name: String): DoubleArray { val a=document.getJSONArray(name); return DoubleArray(a.length()) { a.getDouble(it) } }
    private val mean=vector("mean")
    private val scale=vector("scale")
    private val b1=vector("b1")
    private val w2=vector("w2")
    private val w1=document.getJSONArray("w1").let { rows->Array(rows.length()) { i->val row=rows.getJSONArray(i); DoubleArray(row.length()) { row.getDouble(it) } } }
    private val b2=document.getDouble("b2")
    private val targetMean=document.getDouble("target_mean")
    private val targetScale=document.getDouble("target_scale")
    private val noise=vector("noise_coefficients")
    private val noiseCalibration=document.getDouble("variance_calibration")
    private val eligible=document.optBoolean("eligible_for_fusion",false)
    private val window=ArrayDeque<DoubleArray>()
    private var bucket=-1L
    private var lastNs=0L
    private var count=0
    private val sum=DoubleArray(4)
    @Volatile var latest: Prediction?=null
        private set
    init {
        require(document.getString("format")=="seamlessnav-invariant-mlp-v1")
        require(document.getInt("window_samples")==20 && document.getDouble("feature_rate_hz")==10.0)
        val expected=listOf("mean","std","min","max","last").flatMap { stat->listOf("accel_norm","gravity_norm","linear_norm","gyro_norm").map { "${stat}_${it}" } }
        val names=document.getJSONArray("feature_names")
        require(names.length()==20 && expected.indices.all { names.getString(it)==expected[it] })
        require(mean.size==20 && scale.size==20 && scale.all { it>0 && it.isFinite() })
        require(w1.size==20 && w1.all { it.size==b1.size && it.all(Double::isFinite) } && w2.size==b1.size && noise.size==21)
        require((mean+b1+w2+noise+doubleArrayOf(b2,targetMean,targetScale,noiseCalibration)).all { it.isFinite() })
        require(targetScale>0 && noiseCalibration>0)
    }
    @Synchronized fun addSample(timestampNs: Long, raw: DoubleArray): Prediction? {
        if(raw.size!=12 || raw.any { !it.isFinite() } || timestampNs<=lastNs) return null
        val current=timestampNs/100_000_000L
        if(lastNs>0 && timestampNs-lastNs>250_000_000L) { window.clear(); count=0; sum.fill(0.0); latest=null; bucket=-1 }
        lastNs=timestampNs
        var prediction: Prediction?=null
        if(bucket>=0 && current!=bucket) {
            if(current==bucket+1 && count>0) {
                window.addLast(DoubleArray(4) { sum[it]/count })
                while(window.size>20) window.removeFirst()
                if(window.size==20) { prediction=predictChannels(window.toList(),current*100_000_000L); latest=prediction }
            } else { window.clear(); latest=null }
            count=0; sum.fill(0.0)
        }
        bucket=current
        for(channel in 0..3) sum[channel]+=sqrt((0..2).sumOf { k->raw[channel*3+k]*raw[channel*3+k] })
        count++
        return prediction
    }
    fun predictChannels(rows: List<DoubleArray>, timestampNs: Long): Prediction {
        require(rows.size==20 && rows.all { it.size==4 && it.all(Double::isFinite) })
        val f=DoubleArray(20)
        for(k in 0..3) {
            val avg=rows.sumOf { it[k] }/20
            f[k]=avg; f[4+k]=sqrt(rows.sumOf { (it[k]-avg).pow(2) }/20)
            f[8+k]=rows.minOf { it[k] }; f[12+k]=rows.maxOf { it[k] }; f[16+k]=rows.last()[k]
        }
        val z=DoubleArray(20) { (f[it]-mean[it])/scale[it] }
        val ood=z.any { abs(it)>8 }; for(i in z.indices) z[i]=z[i].coerceIn(-8.0,8.0)
        val hidden=DoubleArray(b1.size) { j->tanh(b1[j]+z.indices.sumOf { i->z[i]*w1[i][j] }) }
        val speed=((b2+hidden.indices.sumOf { hidden[it]*w2[it] })*targetScale+targetMean).coerceIn(0.0,60.0)
        val logVar=(noise.last()+z.indices.sumOf { z[it]*noise[it] }).coerceIn(-4.0,8.0)
        return Prediction(timestampNs,speed,(exp(logVar)*noiseCalibration).coerceIn(0.25,400.0),ood,eligible&&!ood,f[6])
    }
    @Synchronized fun reset() { latest=null; window.clear(); bucket=-1; lastNs=0; count=0; sum.fill(0.0) }
}
