package org.sih.seamlessnav

import android.content.Context
import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.os.Handler
import android.os.Looper
import android.util.LruCache
import org.json.JSONObject
import java.io.File
import java.net.HttpURLConnection
import java.net.URL
import java.util.concurrent.Executors

/** Interactive CARTO/OSM viewport requests only. Offline regional maps use the imported OSM graph. */
class OsmTileStore(context: Context, private val changed: () -> Unit) {
    data class Key(val z: Int, val x: Int, val y: Int) { val id get() = "$z/$x/$y" }
    private val handler = Handler(Looper.getMainLooper())
    private val executor = Executors.newFixedThreadPool(2)
    private val cartoKey = BuildConfig.CARTO_BASEMAP_KEY.trim()
    private val usingCarto = cartoKey.isNotEmpty()
    private val directory = File(context.cacheDir, if (usingCarto) "carto_voyager_tiles_v1" else "osm_tiles_v2").apply { mkdirs() }
    private val memory = object : LruCache<String, Bitmap>(16 * 1024 * 1024) {
        override fun sizeOf(key: String, value: Bitmap) = value.allocationByteCount
    }
    private val due = mutableMapOf<String, Long>()
    private val pending = mutableSetOf<String>()
    private var closed = false
    private var writes = 0
    var status = if (usingCarto) "CARTO Voyager" else "OpenStreetMap"
        private set

    // Called on the UI thread; workers never mutate/recycle a bitmap being drawn.
    fun get(key: Key): Bitmap? = memory.get(key.id)

    fun request(keys: List<Key>, online: Boolean) {
        if (closed || !online) return
        val now = System.currentTimeMillis()
        for (key in keys) {
            if (key.id in pending || (now < (due[key.id] ?: 0L) && get(key) != null)) continue
            if (get(key) == null && now < (due[key.id] ?: 0L)) continue
            // Bound both active and queued requests while the user drags the map.
            if (pending.size >= 12) break
            pending.add(key.id)
            executor.execute { load(key) }
        }
    }

    private fun load(key: Key) {
        val file = File(directory, key.id.replace('/', '_') + ".png")
        val metaFile = File(directory, file.name + ".json")
        var bitmap: Bitmap? = null
        var expiry = System.currentTimeMillis() + 30_000L
        var message = "Tiles unavailable · retrying"
        try {
            val meta = try { JSONObject(metaFile.readText()) } catch (_: Exception) { JSONObject() }
            bitmap = if (file.isFile) BitmapFactory.decodeFile(file.path) else null
            if (bitmap != null && meta.optLong("expires") > System.currentTimeMillis()) {
                expiry = meta.getLong("expires")
                message = "OSM · cached viewport"
            } else {
                val tileUrl = if (usingCarto) {
                    "https://basemaps.cartocdn.com/rastertiles/voyager/${key.id}.png?key=$cartoKey"
                } else {
                    "https://tile.openstreetmap.org/${key.id}.png"
                }
                val connection = URL(tileUrl).openConnection() as HttpURLConnection
                try {
                    connection.connectTimeout = 5000
                    connection.readTimeout = 5000
                    connection.setRequestProperty("User-Agent", "NAVIS/0.4 (org.sih.seamlessnav Android)")
                    connection.setRequestProperty("Accept", "image/png")
                    if (bitmap != null) {
                        meta.optString("etag").takeIf { it.isNotBlank() }?.let { connection.setRequestProperty("If-None-Match", it) }
                        meta.optString("modified").takeIf { it.isNotBlank() }?.let { connection.setRequestProperty("If-Modified-Since", it) }
                    }
                    val response = connection.responseCode
                    require(response == 200 || (response == 304 && bitmap != null)) { "HTTP $response" }
                    if (response == 200) {
                        bitmap = connection.inputStream.use { BitmapFactory.decodeStream(it) }
                        require(bitmap != null && bitmap!!.width == 256 && bitmap!!.height == 256) { "Invalid tile" }
                    }
                    val cc = connection.getHeaderField("Cache-Control").orEmpty().lowercase()
                    val age = connection.getHeaderField("Age")?.toLongOrNull() ?: 0L
                    val maxAge = Regex("max-age\\s*=\\s*(\\d+)").find(cc)?.groupValues?.get(1)?.toLongOrNull()
                    expiry = when {
                        "no-cache" in cc || "no-store" in cc -> System.currentTimeMillis()
                        maxAge != null -> System.currentTimeMillis() + (maxAge - age).coerceAtLeast(0L) * 1000L
                        else -> connection.getHeaderFieldDate("Expires", System.currentTimeMillis() + 7L * 86400 * 1000)
                    }
                    if ("no-store" !in cc) {
                        if (response == 200) {
                            val temporary = File(directory, file.name + ".tmp")
                            temporary.outputStream().use { bitmap!!.compress(Bitmap.CompressFormat.PNG, 100, it) }
                            if (!temporary.renameTo(file)) { temporary.copyTo(file, overwrite = true); temporary.delete() }
                        }
                        JSONObject().put("expires", expiry)
                            .put("etag", connection.getHeaderField("ETag") ?: meta.optString("etag"))
                            .put("modified", connection.getHeaderField("Last-Modified") ?: meta.optString("modified"))
                            .let { metaFile.writeText(it.toString()) }
                    } else { file.delete(); metaFile.delete() }
                    message = if (usingCarto) "CARTO Voyager · online" else "OpenStreetMap · online"
                } finally { connection.disconnect() }
            }
        } catch (_: Exception) {
            if (bitmap != null) message = "OSM cached view · connection unavailable"
        }
        if (++writes % 64 == 0) trimDisk()
        val result = bitmap
        handler.post {
            if (!closed) {
                pending.remove(key.id)
                due[key.id] = expiry
                if (result != null) memory.put(key.id, result)
                status = message
                if (due.size > 4096) due.keys.filter { memory.get(it) == null && it !in pending }.take(2048).forEach { due.remove(it) }
                changed()
            }
        }
    }

    private fun trimDisk() {
        val files = directory.listFiles()?.filter { it.extension == "png" }?.sortedBy { it.lastModified() } ?: return
        var bytes = files.sumOf { it.length() }
        for (file in files) {
            if (bytes <= 128L * 1024 * 1024) break
            val length = file.length()
            if (file.delete()) { bytes -= length; File(directory, file.name + ".json").delete() }
        }
    }

    fun close() { closed = true; executor.shutdownNow(); memory.evictAll(); handler.removeCallbacksAndMessages(null) }
}
