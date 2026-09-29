package org.sih.seamlessnav

import org.json.JSONObject
import java.net.HttpURLConnection
import java.net.URL

/** Public OSRM demo router. Coordinates are sent only when the user explicitly requests a route. */
object PublicOsrmRouter {
    data class Step(val atMeters: Double, val instruction: String, val roadName: String)
    data class Route(
        val points: List<Pair<Double, Double>>,
        val distanceM: Double,
        val durationS: Double,
        val steps: List<Step>
    )

    fun route(startLat: Double, startLon: Double, endLat: Double, endLon: Double): Route {
        require(startLat in -90.0..90.0 && endLat in -90.0..90.0)
        require(startLon in -180.0..180.0 && endLon in -180.0..180.0)
        val url = URL(
            "https://router.project-osrm.org/route/v1/driving/" +
                "$startLon,$startLat;$endLon,$endLat?overview=full&geometries=geojson&steps=true"
        )
        val connection = (url.openConnection() as HttpURLConnection).apply {
            requestMethod = "GET"
            connectTimeout = 8_000
            readTimeout = 12_000
            setRequestProperty("Accept", "application/json")
            setRequestProperty("User-Agent", "NAVIS/0.1 Android navigation prototype")
        }
        try {
            val code = connection.responseCode
            val stream = if (code in 200..299) connection.inputStream else connection.errorStream
            val body = stream.bufferedReader().use { it.readText() }
            if (code !in 200..299) error("Routing service HTTP $code")
            val root = JSONObject(body)
            if (root.optString("code") != "Ok") error(root.optString("message", root.optString("code", "No route found")))
            val route = root.getJSONArray("routes").getJSONObject(0)
            val coordinates = route.getJSONObject("geometry").getJSONArray("coordinates")
            val points = buildList {
                for (index in 0 until coordinates.length()) {
                    val pair = coordinates.getJSONArray(index)
                    add(pair.getDouble(1) to pair.getDouble(0)) // GeoJSON uses longitude, latitude.
                }
            }
            require(points.size >= 2) { "Routing service returned no usable road geometry" }
            val steps = mutableListOf<Step>()
            val legs = route.optJSONArray("legs")
            var along = 0.0
            if (legs != null) for (legIndex in 0 until legs.length()) {
                val legSteps = legs.getJSONObject(legIndex).optJSONArray("steps") ?: continue
                for (stepIndex in 0 until legSteps.length()) {
                    val item = legSteps.getJSONObject(stepIndex)
                    val maneuver = item.optJSONObject("maneuver")
                    val kind = maneuver?.optString("type").orEmpty()
                    val modifier = maneuver?.optString("modifier").orEmpty()
                    val instruction = when {
                        kind == "arrive" -> "Arrive at destination"
                        kind == "depart" -> "Head out"
                        kind == "roundabout" || kind == "rotary" -> "Enter the roundabout"
                        modifier.isNotBlank() -> "Turn ${modifier.replace('_', ' ')}"
                        else -> "Continue"
                    }
                    steps += Step(along, instruction, item.optString("name"))
                    along += item.optDouble("distance", 0.0).coerceAtLeast(0.0)
                }
            }
            return Route(points, route.getDouble("distance"), route.getDouble("duration"), steps)
        } finally {
            connection.disconnect()
        }
    }
}
