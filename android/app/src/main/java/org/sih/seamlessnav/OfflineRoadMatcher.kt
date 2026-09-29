package org.sih.seamlessnav

import org.json.JSONObject
import java.io.File
import java.util.PriorityQueue
import kotlin.math.abs
import kotlin.math.ceil
import kotlin.math.cos
import kotlin.math.exp
import kotlin.math.floor
import kotlin.math.hypot
import kotlin.math.ln
import kotlin.math.max
import kotlin.math.min

/**
 * On-device confidence-gated matcher for the compact JSON emitted by
 * scripts/prepare_offline_osm.py. It returns a separate map hypothesis; callers
 * must preserve the original GNSS/INS state. A bounded beam HMM combines
 * position, heading and directed OSM graph continuity.
 */
class OfflineRoadMatcher private constructor(
    private val originLat: Double,
    private val originLon: Double,
    private val cellSizeM: Double,
    private val arcs: List<RoadArc>,
    val attribution: String
) {
    fun geographicOrigin(): Pair<Double, Double> = originLat to originLon
    data class RenderableRoadSegment(
        val startEastM: Double, val startNorthM: Double,
        val endEastM: Double, val endNorthM: Double,
        val wayId: String, val name: String, val highway: String,
        val tunnel: String, val bridge: String, val layer: Int
    )

    data class Result(
        val accepted: Boolean,
        val latitudeDeg: Double?,
        val longitudeDeg: Double?,
        val roadId: String?,
        val distanceM: Double?,
        val confidence: Double,
        val reason: String,
        val name: String = "",
        val tunnel: String = "",
        val bridge: String = ""
    )

    data class RouteLeg(val startEastM: Double, val startNorthM: Double, val endEastM: Double, val endNorthM: Double,
                        val lengthM: Double, val name: String, val bearingDeg: Double,
                        val tunnel: String, val bridge: String)
    data class RouteManeuver(val atMeters: Double, val instruction: String, val roadName: String)
    data class RoutePlan(val legs: List<RouteLeg>, val distanceM: Double,
                         val destinationLatitudeDeg: Double, val destinationLongitudeDeg: Double,
                         val maneuvers: List<RouteManeuver>)
    data class RouteGuidance(val distanceRemainingM: Double, val distanceToManeuverM: Double?,
                             val instruction: String, val roadName: String, val deviationM: Double)
    data class RoadDestination(val roadId: String, val name: String, val latitudeDeg: Double, val longitudeDeg: Double)

    private data class RoadArc(
        val wayId: String,
        val startE: Double,
        val startN: Double,
        val endE: Double,
        val endN: Double,
        val lengthM: Double,
        val bearingDeg: Double,
        val name: String,
        val highway: String,
        val tunnel: String,
        val bridge: String,
        val layer: Int,
        val segmentId: Int,
        val startNode: String,
        val endNode: String
    )

    private data class Candidate(
        val arc: RoadArc,
        val east: Double,
        val north: Double,
        val fraction: Double,
        val distanceM: Double,
        val logScore: Double
    )

    private data class BeamState(val candidate: Candidate, val logScore: Double)
    private data class QueueNode(val id: String, val distanceM: Double)

    private val grid = HashMap<Long, MutableList<Int>>()
    private val adjacency = HashMap<String, MutableList<Pair<String, Double>>>()
    private val adjacencyArcs = HashMap<String, MutableList<RoadArc>>()
    private val distanceCache = LinkedHashMap<String, Double?>(256, 0.75f, true)
    private var beam: List<BeamState> = emptyList()
    private var lastObservation: PhoneNavigationState? = null

    init {
        require(cellSizeM > 0.0 && cellSizeM.isFinite()) { "offline map cell size is invalid" }
        arcs.forEachIndexed { index, arc ->
            adjacency.getOrPut(arc.startNode) { ArrayList() }.add(arc.endNode to arc.lengthM)
            adjacencyArcs.getOrPut(arc.startNode) { ArrayList() }.add(arc)
            val minX = cell(min(arc.startE, arc.endE)); val maxX = cell(max(arc.startE, arc.endE))
            val minY = cell(min(arc.startN, arc.endN)); val maxY = cell(max(arc.startN, arc.endN))
            for (x in minX..maxX) for (y in minY..maxY) {
                grid.getOrPut(key(x, y)) { ArrayList() }.add(index)
            }
        }
    }

    @Synchronized
    fun reset() { beam = emptyList(); lastObservation = null; distanceCache.clear() }

    @Synchronized
    fun match(state: PhoneNavigationState): Result {
        if (lastObservation != null && state.timestampNs <= lastObservation!!.timestampNs) {
            return Result(false, null, null, null, null, 0.0, "non-increasing map observation timestamp")
        }
        if (state.mode == "DEAD_RECKONING" && state.speedMps >= 2.0 && !state.headingReferenceValid) {
            beam = emptyList()
            lastObservation = state
            return Result(false, null, null, null, null, 0.0, "dead-reckoning heading has no GNSS-course reference")
        }
        val (east, north) = toLocal(state.latitudeDeg, state.longitudeDeg)
        val sigma = state.horizontalSigmaM.coerceIn(3.0, 50.0)
        val radius = max(30.0, min(100.0, 3.0 * sigma))
        val cellRadius = ceil(radius / cellSizeM).toInt()
        val candidateIndices = HashSet<Int>()
        val gx = cell(east); val gy = cell(north)
        for (x in gx - cellRadius..gx + cellRadius) for (y in gy - cellRadius..gy + cellRadius) {
            grid[key(x, y)]?.let(candidateIndices::addAll)
        }
        val scored = ArrayList<Candidate>()
        for (index in candidateIndices) {
            val arc = arcs[index]
            val dx = arc.endE - arc.startE; val dy = arc.endN - arc.startN
            val fraction = ((east - arc.startE) * dx + (north - arc.startN) * dy)
                .div(arc.lengthM * arc.lengthM).coerceIn(0.0, 1.0)
            val snappedE = arc.startE + fraction * dx
            val snappedN = arc.startN + fraction * dy
            val distance = hypot(east - snappedE, north - snappedN)
            if (distance > radius) continue
            var score = -0.5 * (distance / sigma) * (distance / sigma) - ln(sigma)
            if (state.speedMps >= 2.0 && state.headingReferenceValid) {
                val error = absHeadingDifference(arc.bearingDeg, state.headingDeg)
                score -= 0.5 * (error / 35.0) * (error / 35.0)
            }
            scored.add(Candidate(arc, snappedE, snappedN, fraction, distance, score))
        }
        if (scored.isEmpty()) {
            beam = emptyList()
            lastObservation = state
            return Result(false, null, null, null, null, 0.0, "no road candidate within search radius")
        }
        val candidates = scored.sortedByDescending { it.logScore }.take(MAX_CANDIDATES)
        val previous = lastObservation
        val next = ArrayList<BeamState>()
        if (beam.isEmpty() || previous == null) {
            candidates.forEach { next.add(BeamState(it, it.logScore)) }
        } else {
            val dt = (state.timestampNs - previous.timestampNs) / 1e9
            val prevRaw = toLocal(previous.latitudeDeg, previous.longitudeDeg)
            val observedDistance = hypot(east - prevRaw.first, north - prevRaw.second)
            val expectedDistance = max(observedDistance, max(0.0, state.speedMps) * dt)
            val beta = max(8.0, 0.35 * expectedDistance + previous.horizontalSigmaM + state.horizontalSigmaM)
            val cutoff = expectedDistance + max(50.0, 3.0 * beta)
            for (candidate in candidates) {
                var best = Double.NEGATIVE_INFINITY
                for (old in beam) {
                    val routeDistance = routeDistance(old.candidate, candidate, cutoff) ?: continue
                    val transition = -abs(routeDistance - observedDistance) / beta
                    best = max(best, old.logScore + transition + candidate.logScore)
                }
                if (best.isFinite()) next.add(BeamState(candidate, best))
            }
            // Disconnected junctions and long unmatched gaps restart hypotheses
            // from the raw position; confidence gating still controls acceptance.
            if (next.isEmpty()) candidates.forEach { next.add(BeamState(it, it.logScore)) }
        }
        val ordered = next.sortedByDescending { it.logScore }.take(BEAM_SIZE)
        beam = ordered
        lastObservation = state
        val bestState = ordered.first()
        // Relative beam confidence alone can be near 1 even when every road is implausible.
        if (state.horizontalSigmaM > 50.0 || bestState.candidate.distanceM > max(10.0, 3.0*state.horizontalSigmaM)) {
            beam = emptyList()
            return Result(false, null, null, null, bestState.candidate.distanceM, 0.0, "absolute road distance or uncertainty gate")
        }
        val maxScore = bestState.logScore
        val weights = ordered.map { exp((it.logScore - maxScore).coerceAtLeast(-700.0)) }
        val confidence = weights.first() / weights.sum()
        if (confidence < CONFIDENCE_THRESHOLD) {
            return Result(false, null, null, null, bestState.candidate.distanceM, confidence, "road hypothesis confidence below threshold")
        }
        val best = bestState.candidate
        val latitude = originLat + Math.toDegrees(best.north / EARTH_RADIUS_M)
        val longitude = originLon + Math.toDegrees(best.east / (EARTH_RADIUS_M * max(1e-6, cos(Math.toRadians(originLat)))))
        return Result(true, latitude, longitude, best.arc.wayId, best.distanceM, confidence,
            "confidence-gated road match", best.arc.name, best.arc.tunnel, best.arc.bridge)
    }

    fun renderableRoadSegments(): List<RenderableRoadSegment> = arcs.map { arc ->
        RenderableRoadSegment(arc.startE, arc.startN, arc.endE, arc.endN,
            arc.wayId, arc.name, arc.highway, arc.tunnel, arc.bridge, arc.layer)
    }

    /** Plan along directed OSM arcs; rejects endpoints outside this imported region. */
    fun planRoute(startLatitudeDeg: Double, startLongitudeDeg: Double,
                  destinationLatitudeDeg: Double, destinationLongitudeDeg: Double): RoutePlan? {
        if (listOf(startLatitudeDeg,startLongitudeDeg,destinationLatitudeDeg,destinationLongitudeDeg).any { !it.isFinite() }) return null
        if (startLatitudeDeg !in -90.0..90.0 || destinationLatitudeDeg !in -90.0..90.0 ||
            startLongitudeDeg !in -180.0..180.0 || destinationLongitudeDeg !in -180.0..180.0) return null
        val (startE,startN)=toLocal(startLatitudeDeg,startLongitudeDeg)
        val (destE,destN)=toLocal(destinationLatitudeDeg,destinationLongitudeDeg)
        data class Snap(val arc: RoadArc,val e: Double,val n: Double,val fraction: Double,val distance: Double)
        fun nearest(e: Double,n: Double): List<Snap> = arcs.mapNotNull { arc ->
            val dx=arc.endE-arc.startE; val dy=arc.endN-arc.startN
            val f=((e-arc.startE)*dx+(n-arc.startN)*dy)/(arc.lengthM*arc.lengthM)
            val clamped=f.coerceIn(0.0,1.0); val pe=arc.startE+clamped*dx; val pn=arc.startN+clamped*dy
            val d=hypot(e-pe,n-pn)
            if(d<=100.0) Snap(arc,pe,pn,clamped,d) else null
        }.sortedBy { it.distance }.take(8)
        val starts=nearest(startE,startN); val destinations=nearest(destE,destN)
        if(starts.isEmpty() || destinations.isEmpty()) return null
        var bestCost=Double.POSITIVE_INFINITY
        var bestLegs: List<RouteLeg>?=null
        for (start in starts) {
            data class Previous(val node: String,val arc: RoadArc)
            val distances=HashMap<String,Double>(); val previous=HashMap<String,Previous>()
            val queue=PriorityQueue<QueueNode>(compareBy { it.distanceM })
            distances[start.arc.endNode]=0.0;queue.add(QueueNode(start.arc.endNode,0.0))
            while(queue.isNotEmpty()) {
                val item=queue.poll() ?: break
                if(item.distanceM!=distances[item.id]) continue
                for(arc in adjacencyArcs[item.id].orEmpty()) {
                    val next=item.distanceM+arc.lengthM
                    if(next<(distances[arc.endNode] ?: Double.POSITIVE_INFINITY)) {
                        distances[arc.endNode]=next;previous[arc.endNode]=Previous(item.id,arc);queue.add(QueueNode(arc.endNode,next))
                    }
                }
            }
            for(destination in destinations) {
                val connectorStart=hypot(start.e-startE,start.n-startN)
                val connectorEnd=hypot(destination.e-destE,destination.n-destN)
                var roadDistance: Double
                var pathArcs: List<RoadArc>
                if(start.arc.segmentId==destination.arc.segmentId && destination.fraction>=start.fraction) {
                    roadDistance=(destination.fraction-start.fraction)*start.arc.lengthM
                    pathArcs=emptyList()
                } else {
                    val middle=distances[destination.arc.startNode] ?: continue
                    val initial=(1.0-start.fraction)*start.arc.lengthM
                    val terminal=destination.fraction*destination.arc.lengthM
                    roadDistance=initial+middle+terminal
                    val reversed=ArrayList<RoadArc>();var node=destination.arc.startNode
                    while(node!=start.arc.endNode) {
                        val prev=previous[node] ?: break
                        reversed.add(prev.arc);node=prev.node
                    }
                    if(node!=start.arc.endNode) continue
                    pathArcs=reversed.asReversed()
                }
                val total=connectorStart+roadDistance+connectorEnd
                if(total<20.0 || total>bestCost) continue
                val legs=ArrayList<RouteLeg>()
                if(connectorStart>2) legs.add(RouteLeg(startE,startN,start.e,start.n,connectorStart,"Off road",bearing(startE,startN,start.e,start.n),"",""))
                if(pathArcs.isEmpty()) {
                    val length=roadDistance
                    if(length>0.5) legs.add(routeLeg(start.e,start.n,destination.e,destination.n,start.arc,length))
                } else {
                    val initial=(1.0-start.fraction)*start.arc.lengthM
                    if(initial>0.5) legs.add(routeLeg(start.e,start.n,start.arc.endE,start.arc.endN,start.arc,initial))
                    pathArcs.forEach { legs.add(routeLeg(it.startE,it.startN,it.endE,it.endN,it,it.lengthM)) }
                    val terminal=destination.fraction*destination.arc.lengthM
                    if(terminal>0.5) legs.add(routeLeg(destination.arc.startE,destination.arc.startN,destination.e,destination.n,destination.arc,terminal))
                }
                if(connectorEnd>2) legs.add(RouteLeg(destination.e,destination.n,destE,destN,connectorEnd,"Off road",bearing(destination.e,destination.n,destE,destN),"",""))
                if(legs.isNotEmpty()) { bestCost=total;bestLegs=legs }
            }
        }
        val legs=bestLegs ?: return null
        val turns=ArrayList<RouteManeuver>();var along=0.0
        for(i in 1 until legs.size) {
            val previous=legs[i-1];val next=legs[i]
            val delta=((next.bearingDeg-previous.bearingDeg+540.0)%360.0)-180.0
            val roadChanged=next.name.isNotBlank() && next.name!="Off road" && next.name!=previous.name
            if(abs(delta)>=30.0 || roadChanged) {
                val instruction=when { abs(delta)>=150->"Make a U-turn";delta>=30->"Turn right";delta<=-30->"Turn left";else->"Continue" }
                turns.add(RouteManeuver(along,instruction,next.name))
            }
            along+=previous.lengthM
        }
        return RoutePlan(legs,bestCost,destinationLatitudeDeg,destinationLongitudeDeg,turns)
    }

    /** Search named roads in the imported region; this is intentionally not an online geocoder. */
    @Synchronized
    fun searchRoadNames(query: String, nearLatitudeDeg: Double, nearLongitudeDeg: Double): List<RoadDestination> {
        val needle = query.trim().lowercase()
        if (needle.length < 2 || !nearLatitudeDeg.isFinite() || !nearLongitudeDeg.isFinite()) return emptyList()
        val (nearE, nearN) = toLocal(nearLatitudeDeg, nearLongitudeDeg)
        val seen = HashSet<String>()
        return arcs.asSequence()
            .filter { it.name.isNotBlank() && it.name.lowercase().contains(needle) }
            .sortedBy { hypot((it.startE+it.endE)/2.0-nearE, (it.startN+it.endN)/2.0-nearN) }
            .filter { seen.add(it.wayId) }
            .take(12)
            .map { arc ->
                val east=(arc.startE+arc.endE)/2.0;val north=(arc.startN+arc.endN)/2.0
                RoadDestination(arc.wayId, arc.name,
                    originLat + Math.toDegrees(north / EARTH_RADIUS_M),
                    originLon + Math.toDegrees(east / (EARTH_RADIUS_M * max(1e-6, cos(Math.toRadians(originLat)))))
                )
            }.toList()
    }

    /** Current route progress and the next maneuver from an absolute navigation estimate. */
    fun guidance(route: RoutePlan, latitudeDeg: Double, longitudeDeg: Double, minProgressM: Double = 0.0): RouteGuidance {
        val (e,n)=toLocal(latitudeDeg,longitudeDeg);var along=0.0;var bestDistance=Double.POSITIVE_INFINITY;var bestAlong=minProgressM
        for(leg in route.legs) {
            val dx=leg.endEastM-leg.startEastM;val dy=leg.endNorthM-leg.startNorthM
            val f=(((e-leg.startEastM)*dx+(n-leg.startNorthM)*dy)/(leg.lengthM*leg.lengthM)).coerceIn(0.0,1.0)
            val d=hypot(e-(leg.startEastM+dx*f),n-(leg.startNorthM+dy*f))
            val progress=along+f*leg.lengthM
            if(d<bestDistance && progress>=minProgressM-15.0) {bestDistance=d;bestAlong=max(minProgressM,progress)}
            along+=leg.lengthM
        }
        val maneuver=route.maneuvers.firstOrNull { it.atMeters>=bestAlong+2.0 }
        return if(maneuver==null) RouteGuidance(max(0.0,route.distanceM-bestAlong),null,"Arrive at destination","",bestDistance)
        else RouteGuidance(max(0.0,route.distanceM-bestAlong),max(0.0,maneuver.atMeters-bestAlong),maneuver.instruction,maneuver.roadName,bestDistance)
    }

    private fun routeLeg(e1: Double,n1: Double,e2: Double,n2: Double,arc: RoadArc,length: Double)=
        RouteLeg(e1,n1,e2,n2,length,arc.name,arc.bearingDeg,arc.tunnel,arc.bridge)
    private fun bearing(e1:Double,n1:Double,e2:Double,n2:Double)=((Math.toDegrees(kotlin.math.atan2(e2-e1,n2-n1))+360)%360)

    fun toLocal(latitude: Double, longitude: Double): Pair<Double, Double> = Pair(
        Math.toRadians(longitude - originLon) * EARTH_RADIUS_M * cos(Math.toRadians(originLat)),
        Math.toRadians(latitude - originLat) * EARTH_RADIUS_M
    )

    /** Directed shortest road distance between two projected road candidates. */
    private fun routeDistance(start: Candidate, end: Candidate, cutoffM: Double): Double? {
        if (start.arc.segmentId == end.arc.segmentId && end.fraction >= start.fraction) {
            return (end.fraction - start.fraction) * start.arc.lengthM
        }
        val startOffset = (1.0 - start.fraction) * start.arc.lengthM
        val endOffset = end.fraction * end.arc.lengthM
        val remaining = cutoffM - startOffset - endOffset
        if (remaining < 0.0) return null
        val bucket = ceil(remaining / 25.0).toInt()
        val key = "${start.arc.endNode}|${end.arc.startNode}|$bucket"
        if (distanceCache.containsKey(key)) {
            return distanceCache[key]?.let { if (it > remaining) null else startOffset + it + endOffset }
        }
        val pathDistance = shortestPath(start.arc.endNode, end.arc.startNode, bucket * 25.0)
        distanceCache[key] = pathDistance
        while (distanceCache.size > MAX_DISTANCE_CACHE) distanceCache.remove(distanceCache.keys.first())
        return pathDistance?.let { startOffset + it + endOffset }
    }

    private fun shortestPath(from: String, to: String, cutoffM: Double): Double? {
        if (from == to) return 0.0
        val best = HashMap<String, Double>()
        val queue = PriorityQueue<QueueNode>(compareBy { it.distanceM })
        best[from] = 0.0
        queue.add(QueueNode(from, 0.0))
        while (queue.isNotEmpty()) {
            val current = queue.poll() ?: break
            if (current.distanceM != best[current.id]) continue
            if (current.id == to) return current.distanceM
            if (current.distanceM > cutoffM) continue
            adjacency[current.id].orEmpty().forEach { (neighbor, length) ->
                val distance = current.distanceM + length
                if (distance <= cutoffM && distance < (best[neighbor] ?: Double.POSITIVE_INFINITY)) {
                    best[neighbor] = distance
                    queue.add(QueueNode(neighbor, distance))
                }
            }
        }
        return null
    }

    private fun cell(value: Double): Int = floor(value / cellSizeM).toInt()
    private fun key(x: Int, y: Int): Long = (x.toLong() shl 32) xor (y.toLong() and 0xffffffffL)

    private fun absHeadingDifference(a: Double, b: Double): Double {
        val difference = (a - b) % 360.0
        return kotlin.math.abs((difference + 540.0) % 360.0 - 180.0)
    }

    companion object {
        private const val EARTH_RADIUS_M = 6_378_137.0
        private const val CONFIDENCE_THRESHOLD = 0.80
        private const val MAX_CANDIDATES = 12
        private const val BEAM_SIZE = 16
        private const val MAX_DISTANCE_CACHE = 20_000

        fun load(file: File): OfflineRoadMatcher {
            require(file.length() in 1..25L*1024*1024) { "Road map must be between 1 byte and 25 MB" }
            val document = JSONObject(file.readText(Charsets.UTF_8))
            require(document.optInt("schema_version", -1) == 1 &&
                document.optString("format") == "seamlessnav-offline-roads-v1") {
                "unsupported offline road database format"
            }
            val originLat = document.getDouble("origin_latitude_deg")
            val originLon = document.getDouble("origin_longitude_deg")
            require(originLat in -90.0..90.0 && originLon in -180.0..180.0) { "map projection origin is invalid" }
            val cellSize = document.optDouble("cell_size_m", 60.0)
            require(cellSize.isFinite() && cellSize in 10.0..1000.0) { "invalid map index cell size" }
            val array = document.getJSONArray("arcs")
            require(array.length() in 1..60000) { "Road map must contain 1–60000 directed segments" }
            val arcs = ArrayList<RoadArc>(array.length())
            for (i in 0 until array.length()) {
                val item = array.getJSONObject(i)
                val startE = item.getDouble("start_e_m"); val startN = item.getDouble("start_n_m")
                val endE = item.getDouble("end_e_m"); val endN = item.getDouble("end_n_m")
                val length = item.getDouble("length_m")
                require(listOf(startE,startN,endE,endN).all { it.isFinite() && abs(it)<=200000.0 }) { "Road map must be a finite local region" }
                require(hypot(endE-startE,endN-startN) <= 10000.0) { "Road segment too long for regional map index" }
                require(length > 0.0 && length.isFinite()) { "map contains an invalid road segment" }
                arcs.add(RoadArc(item.getString("way_id"), startE, startN, endE, endN, length,
                    item.getDouble("bearing_deg"), item.optString("name", ""),
                    item.optString("highway", ""), item.optString("tunnel", ""),
                    item.optString("bridge", ""), item.optInt("layer", 0), item.optInt("segment_id", i),
                    item.getString("start_node"), item.getString("end_node")))
            }
            require(arcs.isNotEmpty()) { "offline map contains no road segments" }
            return OfflineRoadMatcher(originLat, originLon, cellSize, arcs,
                document.optString("attribution", "© OpenStreetMap contributors; ODbL 1.0"))
        }
    }
}
