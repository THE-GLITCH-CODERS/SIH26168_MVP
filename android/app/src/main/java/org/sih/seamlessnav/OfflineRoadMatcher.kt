package org.sih.seamlessnav

import org.json.JSONObject
import java.io.File
import kotlin.math.ceil
import kotlin.math.cos
import kotlin.math.exp
import kotlin.math.floor
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min

/**
 * On-device confidence-gated matcher for the compact JSON emitted by
 * scripts/prepare_offline_osm.py. It returns a separate map hypothesis; callers
 * must preserve the original GNSS/INS state. The Android implementation uses
 * spatial candidates and emission scores. The richer graph-continuity HMM is
 * available in navcore/map_matching.py for offline replay and edge evaluation.
 */
class OfflineRoadMatcher private constructor(
    private val originLat: Double,
    private val originLon: Double,
    private val cellSizeM: Double,
    private val arcs: List<RoadArc>,
    val attribution: String
) {
    data class Result(
        val accepted: Boolean,
        val latitudeDeg: Double?,
        val longitudeDeg: Double?,
        val roadId: String?,
        val distanceM: Double?,
        val confidence: Double,
        val reason: String
    )

    private data class RoadArc(
        val wayId: String,
        val startE: Double,
        val startN: Double,
        val endE: Double,
        val endN: Double,
        val lengthM: Double,
        val bearingDeg: Double
    )

    private data class Candidate(
        val arc: RoadArc,
        val east: Double,
        val north: Double,
        val distanceM: Double,
        val logScore: Double
    )

    private val grid = HashMap<Long, MutableList<Int>>()

    init {
        require(cellSizeM > 0.0 && cellSizeM.isFinite()) { "offline map cell size is invalid" }
        arcs.forEachIndexed { index, arc ->
            val minX = cell(min(arc.startE, arc.endE)); val maxX = cell(max(arc.startE, arc.endE))
            val minY = cell(min(arc.startN, arc.endN)); val maxY = cell(max(arc.startN, arc.endN))
            for (x in minX..maxX) for (y in minY..maxY) {
                grid.getOrPut(key(x, y)) { ArrayList() }.add(index)
            }
        }
    }

    fun match(state: PhoneNavigationState): Result {
        val (east, north) = toLocal(state.latitudeDeg, state.longitudeDeg)
        val sigma = state.horizontalSigmaM.coerceIn(3.0, 50.0)
        val radius = max(30.0, min(100.0, 3.0 * sigma))
        val cellRadius = ceil(radius / cellSizeM).toInt()
        val candidates = HashSet<Int>()
        val gx = cell(east); val gy = cell(north)
        for (x in gx - cellRadius..gx + cellRadius) for (y in gy - cellRadius..gy + cellRadius) {
            grid[key(x, y)]?.let(candidates::addAll)
        }
        val scored = ArrayList<Candidate>()
        for (index in candidates) {
            val arc = arcs[index]
            val dx = arc.endE - arc.startE; val dy = arc.endN - arc.startN
            val fraction = ((east - arc.startE) * dx + (north - arc.startN) * dy)
                .div(arc.lengthM * arc.lengthM).coerceIn(0.0, 1.0)
            val snappedE = arc.startE + fraction * dx
            val snappedN = arc.startN + fraction * dy
            val distance = hypot(east - snappedE, north - snappedN)
            if (distance > radius) continue
            var score = -0.5 * (distance / sigma) * (distance / sigma) - kotlin.math.ln(sigma)
            if (state.speedMps >= 2.0) {
                val error = absHeadingDifference(arc.bearingDeg, state.headingDeg)
                score -= 0.5 * (error / 35.0) * (error / 35.0)
            }
            scored.add(Candidate(arc, snappedE, snappedN, distance, score))
        }
        if (scored.isEmpty()) return Result(false, null, null, null, null, 0.0, "no road candidate within search radius")
        // Treat adjacent OSM segments from one named/OSM way as a single road
        // hypothesis so segment boundaries do not dilute confidence.
        val roadHypotheses = scored.groupBy { it.arc.wayId }.values
            .map { segments -> segments.maxBy { it.logScore } }
            .sortedByDescending { it.logScore }
        val top = roadHypotheses.take(12)
        val maxScore = top.first().logScore
        val weightSum = top.sumOf { exp((it.logScore - maxScore).coerceAtLeast(-700.0)) }
        val confidence = exp((top.first().logScore - maxScore).coerceAtLeast(-700.0)) / weightSum
        if (confidence < CONFIDENCE_THRESHOLD) {
            return Result(false, null, null, null, top.first().distanceM, confidence, "road hypothesis confidence below threshold")
        }
        val best = top.first()
        val latitude = originLat + Math.toDegrees(best.north / EARTH_RADIUS_M)
        val longitude = originLon + Math.toDegrees(best.east / (EARTH_RADIUS_M * max(1e-6, cos(Math.toRadians(originLat)))))
        return Result(true, latitude, longitude, best.arc.wayId, best.distanceM, confidence, "confidence-gated road match")
    }

    private fun toLocal(latitude: Double, longitude: Double): Pair<Double, Double> = Pair(
        Math.toRadians(longitude - originLon) * EARTH_RADIUS_M * cos(Math.toRadians(originLat)),
        Math.toRadians(latitude - originLat) * EARTH_RADIUS_M
    )

    private fun cell(value: Double): Int = floor(value / cellSizeM).toInt()
    private fun key(x: Int, y: Int): Long = (x.toLong() shl 32) xor (y.toLong() and 0xffffffffL)

    private fun absHeadingDifference(a: Double, b: Double): Double {
        val difference = (a - b) % 360.0
        return kotlin.math.abs((difference + 540.0) % 360.0 - 180.0)
    }

    companion object {
        private const val EARTH_RADIUS_M = 6_378_137.0
        private const val CONFIDENCE_THRESHOLD = 0.80

        fun load(file: File): OfflineRoadMatcher {
            val document = JSONObject(file.readText(Charsets.UTF_8))
            require(document.optInt("schema_version", -1) == 1 &&
                document.optString("format") == "seamlessnav-offline-roads-v1") {
                "unsupported offline road database format"
            }
            val originLat = document.getDouble("origin_latitude_deg")
            val originLon = document.getDouble("origin_longitude_deg")
            require(originLat in -90.0..90.0 && originLon in -180.0..180.0) { "map projection origin is invalid" }
            val cellSize = document.optDouble("cell_size_m", 60.0)
            val array = document.getJSONArray("arcs")
            val arcs = ArrayList<RoadArc>(array.length())
            for (i in 0 until array.length()) {
                val item = array.getJSONObject(i)
                val startE = item.getDouble("start_e_m"); val startN = item.getDouble("start_n_m")
                val endE = item.getDouble("end_e_m"); val endN = item.getDouble("end_n_m")
                val length = item.getDouble("length_m")
                require(length > 0.0 && length.isFinite()) { "map contains an invalid road segment" }
                arcs.add(RoadArc(item.getString("way_id"), startE, startN, endE, endN, length,
                    item.getDouble("bearing_deg")))
            }
            require(arcs.isNotEmpty()) { "offline map contains no road segments" }
            return OfflineRoadMatcher(originLat, originLon, cellSize, arcs,
                document.optString("attribution", "© OpenStreetMap contributors; ODbL 1.0"))
        }
    }
}
