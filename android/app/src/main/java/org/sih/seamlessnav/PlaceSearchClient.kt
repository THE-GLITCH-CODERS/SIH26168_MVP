package org.sih.seamlessnav

import org.json.JSONObject
import java.net.HttpURLConnection
import java.net.URL
import java.net.URLEncoder
import java.nio.charset.StandardCharsets

/** User-submitted place search against Photon's public OpenStreetMap-backed demo API. */
object PlaceSearchClient {
    data class Place(
        val name: String,
        val detail: String,
        val latitude: Double,
        val longitude: Double
    ) {
        fun displayName(): String = listOf(name, detail).filter { it.isNotBlank() }.joinToString(" · ")
    }

    @Volatile private var lastRequestMs = 0L

    fun search(query: String): List<Place> {
        val cleaned = query.trim()
        require(cleaned.length >= 2) { "Enter at least two characters to search." }
        synchronized(this) {
            val delay = 1_000L - (System.currentTimeMillis() - lastRequestMs)
            if (delay > 0) Thread.sleep(delay)
            lastRequestMs = System.currentTimeMillis()
        }
        val encoded = URLEncoder.encode(cleaned, StandardCharsets.UTF_8.name())
        val connection = (URL("https://photon.komoot.io/api/?q=$encoded&limit=5&lang=en&countrycode=in")
            .openConnection() as HttpURLConnection).apply {
            requestMethod = "GET"
            connectTimeout = 8_000
            readTimeout = 12_000
            setRequestProperty("Accept", "application/geo+json, application/json")
            setRequestProperty("User-Agent", "NAVIS/0.1 Android SIH navigation prototype")
        }
        try {
            val code = connection.responseCode
            val stream = if (code in 200..299) connection.inputStream else connection.errorStream
            val body = stream?.bufferedReader()?.use { it.readText() }.orEmpty()
            if (code !in 200..299) error("Place search service HTTP $code")
            val features = JSONObject(body).optJSONArray("features") ?: return emptyList()
            return buildList {
                for (index in 0 until features.length()) {
                    val feature = features.optJSONObject(index) ?: continue
                    val coordinates = feature.optJSONObject("geometry")?.optJSONArray("coordinates") ?: continue
                    if (coordinates.length() < 2) continue
                    val longitude = coordinates.optDouble(0, Double.NaN)
                    val latitude = coordinates.optDouble(1, Double.NaN)
                    if (!latitude.isFinite() || !longitude.isFinite() || latitude !in -90.0..90.0 || longitude !in -180.0..180.0) continue
                    val properties = feature.optJSONObject("properties") ?: JSONObject()
                    val name = properties.optString("name").ifBlank {
                        properties.optString("street").ifBlank { properties.optString("city") }
                    }
                    if (name.isBlank()) continue
                    val detail = listOf("street", "district", "city", "state", "country")
                        .map { properties.optString(it).trim() }
                        .filter { it.isNotBlank() && !it.equals(name, ignoreCase = true) }
                        .distinct()
                        .take(3)
                        .joinToString(", ")
                    add(Place(name, detail, latitude, longitude))
                }
            }
        } finally {
            connection.disconnect()
        }
    }
}
