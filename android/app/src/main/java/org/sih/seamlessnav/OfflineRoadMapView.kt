package org.sih.seamlessnav

import android.content.Context
import android.animation.ValueAnimator
import android.graphics.*
import android.location.Location
import android.os.SystemClock
import android.view.MotionEvent
import android.view.ScaleGestureDetector
import android.view.View
import java.util.Locale
import kotlin.math.*

/** OSM viewport. Browsing never initializes or alters the navigation filter. */
class OfflineRoadMapView(context: Context) : View(context) {
    private val prefs = context.getSharedPreferences("map_camera", Context.MODE_PRIVATE)
    private val tiles = OsmTileStore(context) { invalidate() }
    private val paint = Paint(Paint.ANTI_ALIAS_FLAG)
    private val bitmapPaint = Paint(Paint.FILTER_BITMAP_FLAG)
    private var map: OfflineRoadMatcher? = null
    private var roads: List<OfflineRoadMatcher.RenderableRoadSegment> = emptyList()
    private var nav: PhoneNavigationState? = null
    private var hypothesis: OfflineRoadMatcher.Result? = null
    private var route: OfflineRoadMatcher.RoutePlan? = null
    private var onlineRoute: PublicOsrmRouter.Route? = null
    private var routeGuidance: OfflineRoadMatcher.RouteGuidance? = null
    private var destinationTarget: Pair<Double, Double>? = null
    private var routeProgressM = 0.0
    private var destinationPicking = false
    private var searchOverlayVisible = false
    var onDestinationTap: ((Double, Double) -> Unit)? = null
    private var phone: Location? = null
    private var camera: Pair<Double, Double>? = prefs.getString("lat", null)?.toDoubleOrNull()?.let { lat ->
        prefs.getString("lon", null)?.toDoubleOrNull()?.let { lon -> if (lat in -85.0..85.0 && lon in -180.0..180.0) lat to lon else null }
    }
    private var zoom = prefs.getInt("zoom", 4).coerceIn(3, 19)
    private var following = true
    private var offline = false
    private var firstFix = true
    private var centerAnimator: ValueAnimator? = null
    private val raw = ArrayDeque<Pair<Double, Double>>()
    private val matched = ArrayDeque<Pair<Double, Double>?>()
    private var lastX = 0f
    private var lastY = 0f
    private var downX = 0f
    private var downY = 0f
    private var scaleAccumulator = 1f
    private val scaleGesture = ScaleGestureDetector(context, object : ScaleGestureDetector.SimpleOnScaleGestureListener() {
        override fun onScale(detector: ScaleGestureDetector): Boolean {
            scaleAccumulator *= detector.scaleFactor
            if (scaleAccumulator > 1.3f) { zoomBy(1); scaleAccumulator = 1f }
            if (scaleAccumulator < 0.77f) { zoomBy(-1); scaleAccumulator = 1f }
            return true
        }
    })
    private val tileSize get() = dp(256f).toDouble()
    // Keep the live-position marker above the floating navigation sheet.
    private val mapFocusY get() = height * 0.40
    private val refresh = object : Runnable {
        override fun run() { requestTiles(); invalidate(); postDelayed(this, 1000L) }
    }

    fun setMap(matcher: OfflineRoadMatcher) {
        map = matcher
        roads = matcher.renderableRoadSegments()
        if (nav == null && phone == null) { camera = matcher.geographicOrigin(); zoom = 16 }
        requestTiles(); invalidate()
    }
    fun updatePhoneLocation(location: Location?) {
        if (location == null || !location.hasAccuracy() || !location.accuracy.isFinite() || location.accuracy !in 0.01f..250f ||
            location.latitude !in -85.0..85.0 || location.longitude !in -180.0..180.0 ||
            SystemClock.elapsedRealtimeNanos() - location.elapsedRealtimeNanos !in 0L..10_000_000_000L) return
        phone = Location(location)
        if (firstFix) { firstFix = false; zoom = 16; camera = location.latitude to location.longitude }
        else if (following && nav == null) animateCameraTo(location.latitude to location.longitude)
        requestTiles(); invalidate()
    }
    fun updateNavigation(state: PhoneNavigationState, result: OfflineRoadMatcher.Result?) {
        nav = state; hypothesis = result
        route?.let { planned ->
            map?.guidance(planned, state.latitudeDeg, state.longitudeDeg, routeProgressM)?.let { guidance ->
                routeGuidance = guidance
                if (guidance.deviationM <= 60.0) routeProgressM = max(routeProgressM, planned.distanceM - guidance.distanceRemainingM)
            }
        }
        onlineRoute?.let { planned -> routeGuidance = onlineGuidance(planned, state.latitudeDeg, state.longitudeDeg) }
        if (firstFix) { firstFix = false; zoom = 16; camera = state.latitudeDeg to state.longitudeDeg }
        val point = state.latitudeDeg to state.longitudeDeg
        if (following && !firstFix) animateCameraTo(point)
        if (raw.lastOrNull() != point) raw.addLast(point)
        while (raw.size > 3000) raw.removeFirst()
        val mp = if (result?.accepted == true && result.latitudeDeg != null && result.longitudeDeg != null) result.latitudeDeg to result.longitudeDeg else null
        if (matched.lastOrNull() != mp) matched.addLast(mp)
        while (matched.size > 3000) matched.removeFirst()
        requestTiles(); invalidate()
    }
    fun setRoute(value: OfflineRoadMatcher.RoutePlan?) { route=value; onlineRoute=null; routeGuidance=null; routeProgressM=0.0; destinationTarget=null; destinationPicking=false; invalidate() }
    fun setOnlineRoute(value: PublicOsrmRouter.Route?) { onlineRoute=value; route=null; routeGuidance=null; routeProgressM=0.0; destinationTarget=null; destinationPicking=false; invalidate() }
    fun setDestinationTarget(latitude: Double, longitude: Double) {
        if (latitude !in -90.0..90.0 || longitude !in -180.0..180.0) return
        destinationTarget = latitude to longitude
        camera = latitude to longitude
        following = false
        zoom = max(zoom, 14)
        requestTiles(); invalidate()
    }
    fun clearDestinationTarget() { destinationTarget = null; invalidate() }
    fun routeGuidanceSnapshot(): OfflineRoadMatcher.RouteGuidance? = routeGuidance
    fun routeDistanceMeters(): Double? = route?.distanceM ?: onlineRoute?.distanceM
    fun setDestinationPicking(value: Boolean) { destinationPicking=value; invalidate() }
    fun cancelDestinationPicking() { destinationPicking=false; invalidate() }
    fun setSearchOverlayVisible(value: Boolean) { searchOverlayVisible=value; invalidate() }
    fun resetSession() { camera = center(); nav = null; phone = null; hypothesis = null; route=null; onlineRoute=null; routeGuidance=null; routeProgressM=0.0; destinationTarget=null; raw.clear(); matched.clear(); firstFix = true; invalidate() }
    fun recenter() {
        following = true
        val target = nav?.let { it.latitudeDeg to it.longitudeDeg } ?: phone?.let { it.latitude to it.longitude }
        if (target != null) animateCameraTo(target)
        requestTiles(); invalidate()
    }
    fun zoomBy(delta: Int) { zoom = (zoom + delta).coerceIn(3, 19); requestTiles(); invalidate() }
    fun setOffline(value: Boolean) { offline = value; requestTiles(); invalidate() }
    fun close() { removeCallbacks(refresh); tiles.close(); saveCamera() }

    private fun animateCameraTo(target: Pair<Double, Double>) {
        centerAnimator?.cancel()
        val start = camera ?: center()
        centerAnimator = ValueAnimator.ofFloat(0f, 1f).apply {
            duration = 180L
            addUpdateListener { animator ->
                val t = animator.animatedValue as Float
                camera = (start.first + (target.first - start.first) * t) to
                    (start.second + (target.second - start.second) * t)
                postInvalidateOnAnimation()
            }
            start()
        }
    }

    private fun center(): Pair<Double, Double> = if (following) {
        camera ?: nav?.let { it.latitudeDeg to it.longitudeDeg } ?: phone?.let { it.latitude to it.longitude } ?: map?.geographicOrigin() ?: (22.0 to 79.0)
    } else camera ?: (22.0 to 79.0)

    private fun visibleTiles(): List<OsmTileStore.Key> {
        if (width == 0 || height == 0) return emptyList()
        val c = center(); val (cx, cy) = pixel(c.first, c.second)
        val n = 1 shl zoom
        return buildList {
            for (y in floor((cy-mapFocusY)/tileSize).toInt()..floor((cy+height-mapFocusY)/tileSize).toInt()) {
                for (x in floor((cx-width/2.0)/tileSize).toInt()..floor((cx+width/2.0)/tileSize).toInt()) {
                    if (x in 0 until n && y in 0 until n) add(OsmTileStore.Key(zoom,x,y))
                }
            }
        }
    }
    private fun requestTiles() {
        val visible = android.graphics.Rect()
        if (getGlobalVisibleRect(visible) && visible.width() > 0 && visible.height() > 0) {
            tiles.request(visibleTiles(), !offline && isShown)
        }
    }
    override fun onAttachedToWindow() { super.onAttachedToWindow(); removeCallbacks(refresh); post(refresh) }
    override fun onDetachedFromWindow() { removeCallbacks(refresh); saveCamera(); super.onDetachedFromWindow() }
    override fun onSizeChanged(w: Int,h: Int,ow: Int,oh: Int) { super.onSizeChanged(w,h,ow,oh); requestTiles() }
    private fun saveCamera() { val c=center(); prefs.edit().putString("lat",c.first.toString()).putString("lon",c.second.toString()).putInt("zoom",zoom).apply() }

    override fun onTouchEvent(event: MotionEvent): Boolean {
        scaleGesture.onTouchEvent(event)
        when (event.actionMasked) {
            MotionEvent.ACTION_DOWN -> { lastX=event.x; lastY=event.y; downX=event.x; downY=event.y; parent?.requestDisallowInterceptTouchEvent(true) }
            MotionEvent.ACTION_MOVE -> {
                if (!scaleGesture.isInProgress && event.pointerCount==1) {
                    val c=center(); val p=pixel(c.first,c.second)
                    camera=geographic(p.first+lastX-event.x,p.second+lastY-event.y)
                    following=false; requestTiles(); invalidate()
                }
                lastX=event.x; lastY=event.y
            }
            MotionEvent.ACTION_UP -> {
                parent?.requestDisallowInterceptTouchEvent(false)
                if(destinationPicking && hypot(event.x-downX,event.y-downY)<dp(14f) && event.y>dp(34f) && event.y<height-dp(55f)) {
                    val c=center();val p=pixel(c.first,c.second)
                    val point=geographic(p.first+event.x-width/2.0,p.second+event.y-mapFocusY)
                    onDestinationTap?.invoke(point.first,point.second)
                }
                saveCamera(); performClick()
            }
            MotionEvent.ACTION_CANCEL -> { parent?.requestDisallowInterceptTouchEvent(false); saveCamera() }
        }
        return true
    }
    override fun performClick(): Boolean { super.performClick(); return true }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        canvas.drawColor(if(offline) 0xffe3ebed.toInt() else 0xffeef1ed.toInt())
        val c=center(); val cp=pixel(c.first,c.second)
        fun xy(lat: Double,lon: Double): Pair<Float,Float> { val p=pixel(lat,lon); return (width/2.0+p.first-cp.first).toFloat() to (mapFocusY+p.second-cp.second).toFloat() }
        if(!offline) visibleTiles().forEach { key ->
            val x=(width/2.0+key.x*tileSize-cp.first).toFloat(); val y=(mapFocusY+key.y*tileSize-cp.second).toFloat()
            tiles.get(key)?.let { canvas.drawBitmap(it,null,RectF(x,y,x+tileSize.toFloat(),y+tileSize.toFloat()),bitmapPaint) }
        }
        val mpp=2*Math.PI*6378137.0*cos(Math.toRadians(c.first))/(tileSize*(1 shl zoom))
        map?.let { graph ->
            val local=graph.toLocal(c.first,c.second)
            var names=0; val seen=mutableSetOf<String>()
            for(road in roads) {
                val x1=(width/2.0+(road.startEastM-local.first)/mpp).toFloat(); val y1=(mapFocusY-(road.startNorthM-local.second)/mpp).toFloat()
                val x2=(width/2.0+(road.endEastM-local.first)/mpp).toFloat(); val y2=(mapFocusY-(road.endNorthM-local.second)/mpp).toFloat()
                if(max(x1,x2)<0 || min(x1,x2)>width || max(y1,y2)<0 || min(y1,y2)>height) continue
                val tunnel=road.tunnel.isNotBlank() && road.tunnel.lowercase() !in setOf("no","false","0")
                val bridge=road.bridge.isNotBlank() && road.bridge.lowercase() !in setOf("no","false","0")
                paint.color=when { tunnel->0xffbf771a.toInt(); bridge->0xff4477b8.toInt(); else->0xff9aa9ae.toInt() }
                paint.strokeWidth=dp(if(tunnel||bridge) 4f else 2f); paint.style=Paint.Style.STROKE
                paint.pathEffect=if(tunnel) DashPathEffect(floatArrayOf(dp(6f),dp(4f)),0f) else null
                canvas.drawLine(x1,y1,x2,y2,paint); paint.pathEffect=null
                if(offline && zoom>=15 && names<18 && road.name.isNotBlank() && seen.add(road.name)) {
                    text(canvas,road.name,(x1+x2)/2,(y1+y2)/2-dp(4f),9f,0xff4b5c68.toInt()); names++
                }
            }
        }
        route?.let { planned ->
            paint.color=Color.WHITE;paint.style=Paint.Style.STROKE;paint.strokeWidth=dp(9f);paint.pathEffect=null
            val outline=Path()
            planned.legs.forEachIndexed { index, leg ->
                val a=xy(graphicLat(leg.startNorthM),graphicLon(leg.startEastM));val b=xy(graphicLat(leg.endNorthM),graphicLon(leg.endEastM))
                if(index==0) outline.moveTo(a.first,a.second) else outline.lineTo(a.first,a.second)
                outline.lineTo(b.first,b.second)
            }
            canvas.drawPath(outline,paint)
            paint.color=0xff2875ef.toInt();paint.strokeWidth=dp(5f);canvas.drawPath(outline,paint)
            val dest=xy(planned.destinationLatitudeDeg,planned.destinationLongitudeDeg)
            paint.color=Color.WHITE;paint.style=Paint.Style.FILL;canvas.drawCircle(dest.first,dest.second,dp(11f),paint)
            paint.color=0xffdb3f4f.toInt();canvas.drawCircle(dest.first,dest.second,dp(7f),paint)
        }
        onlineRoute?.let { planned ->
            val path = Path()
            planned.points.forEachIndexed { index, point ->
                val p = xy(point.first, point.second)
                if (index == 0) path.moveTo(p.first, p.second) else path.lineTo(p.first, p.second)
            }
            paint.color = Color.WHITE; paint.style = Paint.Style.STROKE; paint.strokeWidth = dp(9f); paint.pathEffect = null
            canvas.drawPath(path, paint)
            paint.color = 0xff2875ef.toInt(); paint.strokeWidth = dp(5f)
            canvas.drawPath(path, paint)
            planned.points.lastOrNull()?.let { point ->
                val dest = xy(point.first, point.second)
                paint.color = Color.WHITE; paint.style = Paint.Style.FILL; canvas.drawCircle(dest.first, dest.second, dp(11f), paint)
                paint.color = 0xffdb3f4f.toInt(); canvas.drawCircle(dest.first, dest.second, dp(7f), paint)
            }
        }
        destinationTarget?.let { target ->
            val p = xy(target.first, target.second)
            paint.style = Paint.Style.FILL
            paint.color = Color.WHITE
            canvas.drawCircle(p.first, p.second, dp(13f), paint)
            paint.color = 0xffd93b4b.toInt()
            canvas.drawCircle(p.first, p.second, dp(9f), paint)
            paint.color = Color.WHITE
            paint.textSize = dp(10f)
            canvas.drawText("D", p.first - paint.measureText("D") / 2, p.second + dp(4f), paint)
        }
        fun track(points: Iterable<Pair<Double,Double>?>,color: Int) {
            val path=Path(); var started=false
            for(point in points) {
                if(point==null) { started=false; continue }
                val p=xy(point.first,point.second)
                if(started) path.lineTo(p.first,p.second) else { path.moveTo(p.first,p.second); started=true }
            }
            paint.color=color; paint.style=Paint.Style.STROKE; paint.strokeWidth=dp(3f); canvas.drawPath(path,paint)
        }
        track(raw,0xffe25757.toInt()); track(matched,0xff087d69.toInt())
        val position=nav?.let { it.latitudeDeg to it.longitudeDeg } ?: phone?.let { it.latitude to it.longitude }
        position?.let { pos ->
            val p=xy(pos.first,pos.second); val sigma=nav?.horizontalSigmaM ?: phone!!.accuracy.toDouble()
            paint.style=Paint.Style.FILL; paint.color=0x243497cf
            canvas.drawCircle(p.first,p.second,(sigma/mpp).coerceIn(1.0,1e6).toFloat(),paint)
            paint.color=Color.WHITE; canvas.drawCircle(p.first,p.second,dp(12f),paint)
            paint.color=when {
                nav?.mode=="DEAD_RECKONING" -> 0xffc47812.toInt()
                nav != null -> 0xff087d69.toInt()
                phone?.provider == android.location.LocationManager.NETWORK_PROVIDER -> 0xffd18a22.toInt()
                else -> 0xff087d69.toInt()
            }
            val heading=nav?.takeIf { it.headingReferenceValid }?.headingDeg
            if(heading==null) canvas.drawCircle(p.first,p.second,dp(8f),paint) else {
                val rad=Math.toRadians(heading); val dx=sin(rad).toFloat(); val dy=-cos(rad).toFloat()
                val path=Path(); path.moveTo(p.first+dx*dp(15f),p.second+dy*dp(15f))
                path.lineTo(p.first-dx*dp(8f)-dy*dp(8f),p.second-dy*dp(8f)+dx*dp(8f))
                path.lineTo(p.first-dx*dp(8f)+dy*dp(8f),p.second-dy*dp(8f)-dx*dp(8f)); path.close(); canvas.drawPath(path,paint)
            }
        }
        hypothesis?.takeIf { it.accepted && it.latitudeDeg!=null && it.longitudeDeg!=null }?.let { m ->
            val p=xy(m.latitudeDeg!!,m.longitudeDeg!!); paint.style=Paint.Style.STROKE; paint.strokeWidth=dp(3f); paint.color=0xff087d69.toInt(); canvas.drawCircle(p.first,p.second,dp(7f),paint)
        }
        if (!searchOverlayVisible) {
            val left = dp(10f)
            val top = dp(8f)
            val right = width - dp(10f)
            val guidance = routeGuidance
            val cardHeight = if (guidance != null) dp(72f) else dp(38f)
            paint.style = Paint.Style.FILL
            paint.color = 0xfafcfcfc.toInt()
            canvas.drawRoundRect(RectF(left, top, right, top + cardHeight), dp(16f), dp(16f), paint)
            paint.color = 0xffdce6ea.toInt()
            paint.style = Paint.Style.STROKE
            paint.strokeWidth = dp(1f)
            canvas.drawRoundRect(RectF(left, top, right, top + cardHeight), dp(16f), dp(16f), paint)
            paint.style = Paint.Style.FILL
            val mode = when {
                destinationPicking -> "TAP MAP TO SET DESTINATION"
                route != null || onlineRoute != null -> "ROUTE ACTIVE · ${"%.1f".format((route?.distanceM ?: onlineRoute?.distanceM ?: 0.0) / 1000)} km"
                destinationTarget != null -> "DESTINATION SET · WAITING FOR ROUTE"
                position == null -> "WAITING FOR LOCATION"
                nav == null && phone?.provider == android.location.LocationManager.NETWORK_PROVIDER -> "NETWORK MAP LOCATION · WAITING FOR GNSS FIX"
                nav == null && phone?.provider == android.location.LocationManager.GPS_PROVIDER -> "PHONE GPS DOT · WAITING FOR ACCEPTED NAV FIX"
                else -> nav?.mode?.replace('_', ' ') ?: "PHONE LOCATION"
            }
            if (guidance == null) {
                text(canvas, mode, left + dp(14f), top + dp(24f), 11f, 0xff173344.toInt())
            } else {
                val icon = RectF(left + dp(10f), top + dp(10f), left + dp(48f), top + dp(48f))
                paint.color = 0xffe5edff.toInt()
                canvas.drawRoundRect(icon, dp(11f), dp(11f), paint)
                paint.color = 0xff2868df.toInt()
                paint.textSize = dp(23f)
                canvas.drawText("↱", icon.left + dp(9f), icon.bottom - dp(7f), paint)
                val distance = guidance.distanceToManeuverM?.let {
                    if (it < 1000) "${it.toInt()} m" else "${"%.1f".format(it / 1000)} km"
                } ?: "Arrive"
                text(canvas, distance, left + dp(58f), top + dp(27f), 20f, 0xff111e2a.toInt())
                text(canvas, guidance.instruction.uppercase(Locale.getDefault()), left + dp(58f), top + dp(45f), 10f, 0xff4c5c68.toInt())
                val road = guidance.roadName.ifBlank { "Follow route" }
                val remaining = if (guidance.distanceRemainingM < 1000) "${guidance.distanceRemainingM.toInt()} m" else "${"%.1f".format(guidance.distanceRemainingM / 1000)} km"
                val rightText = "${remaining} left"
                paint.textSize = dp(10f)
                text(canvas, rightText, right - paint.measureText(rightText) - dp(12f), top + dp(27f), 10f, 0xff52626d.toInt())
                text(canvas, road, left + dp(58f), top + dp(61f), 9f, if (guidance.deviationM > 60.0) 0xffb34b37.toInt() else 0xff647580.toInt())
            }
        }
        paint.style=Paint.Style.FILL; paint.color=0xeefbffff.toInt(); canvas.drawRect(0f,height-dp(20f),width.toFloat(),height.toFloat(),paint)
        val attribution = if (BuildConfig.CARTO_BASEMAP_KEY.isNotBlank()) {
            "© OpenStreetMap contributors · © CARTO"
        } else "© OpenStreetMap contributors"
        text(canvas,attribution,dp(8f),height-dp(6f),9f,0xff344b59.toInt())
    }
    private fun text(canvas: Canvas,value: String,x: Float,y: Float,size: Float,color: Int) {
        paint.style=Paint.Style.FILL; paint.color=color; paint.textSize=dp(size); paint.pathEffect=null
        val count=paint.breakText(value,true,(width-x-dp(6f)).coerceAtLeast(0f),null)
        canvas.drawText(value.take(count),x,y,paint)
    }
    private fun onlineGuidance(route: PublicOsrmRouter.Route, latitude: Double, longitude: Double): OfflineRoadMatcher.RouteGuidance {
        var walked = 0.0
        var nearest = Double.POSITIVE_INFINITY
        var progress = 0.0
        val latScale = 111_320.0
        val lonScale = latScale * cos(Math.toRadians(latitude))
        for (index in 0 until route.points.lastIndex) {
            val a = route.points[index]; val b = route.points[index + 1]
            val ax = (a.second - longitude) * lonScale; val ay = (a.first - latitude) * latScale
            val bx = (b.second - longitude) * lonScale; val by = (b.first - latitude) * latScale
            val dx = bx - ax; val dy = by - ay
            val length = hypot(dx, dy)
            if (length <= 0.01) continue
            val fraction = ((-ax * dx - ay * dy) / (length * length)).coerceIn(0.0, 1.0)
            val distance = hypot(ax + fraction * dx, ay + fraction * dy)
            if (distance < nearest) { nearest = distance; progress = walked + fraction * length }
            walked += length
        }
        val next = route.steps.firstOrNull { it.atMeters > progress + 8.0 }
        return if (next == null) {
            OfflineRoadMatcher.RouteGuidance((route.distanceM - progress).coerceAtLeast(0.0), null, "Arrive at destination", "", nearest)
        } else {
            OfflineRoadMatcher.RouteGuidance(
                (route.distanceM - progress).coerceAtLeast(0.0),
                (next.atMeters - progress).coerceAtLeast(0.0),
                next.instruction,
                next.roadName,
                nearest
            )
        }
    }
    private fun pixel(lat: Double,lon: Double): Pair<Double,Double> {
        val n=tileSize*(1 shl zoom); val s=sin(Math.toRadians(lat.coerceIn(-85.05112878,85.05112878)))
        return (lon+180.0)/360*n to (0.5-ln((1+s)/(1-s))/(4*Math.PI))*n
    }
    private fun geographic(x: Double,y: Double): Pair<Double,Double> {
        val n=tileSize*(1 shl zoom)
        return Math.toDegrees(atan(sinh(Math.PI*(1-2*y.coerceIn(0.0,n)/n)))) to (x/n*360-180).coerceIn(-180.0,180.0)
    }
    private fun graphicLat(north: Double)=map?.let { it.geographicOrigin().first+Math.toDegrees(north/6_378_137.0) } ?: 0.0
    private fun graphicLon(east: Double)=map?.let { val o=it.geographicOrigin();o.second+Math.toDegrees(east/(6_378_137.0*cos(Math.toRadians(o.first)))) } ?: 0.0
    private fun dp(v: Float)=v*resources.displayMetrics.density
}
