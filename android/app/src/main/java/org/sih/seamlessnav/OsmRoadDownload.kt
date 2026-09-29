package org.sih.seamlessnav

import org.json.JSONArray
import org.json.JSONObject
import java.net.HttpURLConnection
import java.net.URL
import java.net.URLEncoder
import kotlin.math.*

/** Small user-requested OSM data extract, independent of the public raster tile cache. */
object OsmRoadDownload {
    fun download(latitude: Double, longitude: Double): JSONObject {
        require(latitude in -80.0..80.0 && longitude in -179.9..179.9)
        val dLat=1500.0/111320.0; val dLon=dLat/cos(Math.toRadians(latitude))
        val query="[out:json][timeout:25];way[highway~\"^(motorway|trunk|primary|secondary|tertiary|unclassified|residential|living_street|service|motorway_link|trunk_link|primary_link|secondary_link|tertiary_link)$\"](${latitude-dLat},${longitude-dLon},${latitude+dLat},${longitude+dLon});out geom;"
        val connection=URL("https://overpass-api.de/api/interpreter").openConnection() as HttpURLConnection
        try {
            connection.requestMethod="POST"; connection.doOutput=true
            connection.connectTimeout=15000; connection.readTimeout=35000
            connection.setRequestProperty("User-Agent","NAVIS/0.4 (org.sih.seamlessnav Android)")
            connection.setRequestProperty("Content-Type","application/x-www-form-urlencoded; charset=UTF-8")
            connection.outputStream.use { it.write(("data="+URLEncoder.encode(query,"UTF-8")).toByteArray(Charsets.UTF_8)) }
            require(connection.responseCode==200) { "Road service unavailable (HTTP ${connection.responseCode}); retry later" }
            val bytes=connection.inputStream.use { input ->
                val output=java.io.ByteArrayOutputStream(); val buffer=ByteArray(8192)
                while(true) { val n=input.read(buffer); if(n<0) break; require(output.size()+n<=10*1024*1024) { "Road extract exceeds 10 MB" }; output.write(buffer,0,n) }
                output.toByteArray()
            }
            return convert(JSONObject(String(bytes,Charsets.UTF_8)),latitude,longitude)
        } finally { connection.disconnect() }
    }

    fun convert(data: JSONObject, lat0: Double, lon0: Double): JSONObject {
        require(!data.has("remark")) { "Road query was incomplete: ${data.optString("remark")}" }
        val nodes=JSONObject(); val arcs=JSONArray(); val elements=data.getJSONArray("elements")
        for(i in 0 until elements.length()) {
            val way=elements.getJSONObject(i); if(way.optString("type")!="way") continue
            val tags=way.optJSONObject("tags") ?: continue
            if(tags.optString("motor_vehicle").lowercase()=="no" || tags.optString("access").lowercase()=="no") continue
            val geometry=way.optJSONArray("geometry") ?: continue
            val ids=way.getJSONArray("nodes"); if(ids.length()!=geometry.length()) continue
            val positions=ArrayList<Pair<Double,Double>>()
            for(k in 0 until geometry.length()) {
                val p=geometry.getJSONObject(k); val lat=p.getDouble("lat"); val lon=p.getDouble("lon")
                require(lat in -90.0..90.0 && lon in -180.0..180.0)
                val e=Math.toRadians(lon-lon0)*6378137*cos(Math.toRadians(lat0)); val n=Math.toRadians(lat-lat0)*6378137
                positions.add(e to n); nodes.put(ids.get(k).toString(),JSONArray(listOf(e,n)))
            }
            val oneway=tags.optString("oneway").lowercase()
            val reverse=oneway=="-1"
            val one=oneway in setOf("yes","true","1","-1") || (oneway !in setOf("no","false","0") && tags.optString("junction")=="roundabout")
            for(k in 0 until positions.size-1) {
                fun arc(from: Int,to: Int) {
                    val a=positions[from]; val b=positions[to]; val length=hypot(b.first-a.first,b.second-a.second)
                    if(length<0.1) return
                    arcs.put(JSONObject().put("segment_id",arcs.length()).put("way_id",way.get("id").toString())
                        .put("start_node",ids.get(from).toString()).put("end_node",ids.get(to).toString())
                        .put("start_e_m",a.first).put("start_n_m",a.second).put("end_e_m",b.first).put("end_n_m",b.second)
                        .put("length_m",length).put("bearing_deg",(Math.toDegrees(atan2(b.first-a.first,b.second-a.second))+360)%360)
                        .put("name",tags.optString("name")).put("highway",tags.optString("highway"))
                        .put("tunnel",tags.optString("tunnel")).put("bridge",tags.optString("bridge")).put("layer",tags.optString("layer").toIntOrNull() ?: 0))
                }
                if(reverse) arc(k+1,k) else { arc(k,k+1); if(!one) arc(k+1,k) }
                require(arcs.length()<=60000) { "Too many road segments; use a smaller desktop extract" }
            }
        }
        require(arcs.length()>0) { "No car roads returned for this area" }
        return JSONObject().put("schema_version",1).put("format","seamlessnav-offline-roads-v1")
            .put("origin_latitude_deg",lat0).put("origin_longitude_deg",lon0).put("cell_size_m",60.0)
            .put("attribution","© OpenStreetMap contributors; ODbL 1.0").put("nodes",nodes).put("arcs",arcs)
            .put("downloaded_utc",java.time.Instant.now().toString()).put("nominal_radius_m",1500)
    }
}
