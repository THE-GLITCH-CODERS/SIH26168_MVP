package org.sih.seamlessnav

import android.Manifest
import android.app.Activity
import android.content.Context
import android.content.Intent
import android.content.pm.PackageManager
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.location.GnssStatus
import android.os.Bundle
import android.os.Handler
import android.os.HandlerThread
import android.os.Looper
import android.os.Build
import android.graphics.Color
import android.graphics.drawable.GradientDrawable
import android.view.WindowManager
import android.view.WindowInsets
import android.widget.Button
import android.widget.LinearLayout
import android.widget.ScrollView
import android.widget.TextView
import android.widget.Toast
import java.io.BufferedWriter
import java.io.File
import java.io.FileInputStream
import java.io.FileOutputStream
import java.io.OutputStreamWriter
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicInteger
import java.util.zip.ZipEntry
import java.util.zip.ZipOutputStream
import org.json.JSONArray
import org.json.JSONObject
import kotlin.math.abs

/** First phone milestone: inspect sensors and capture synchronized, raw observations. */
class MainActivity : Activity(), SensorEventListener, LocationListener {
    private data class RateStats(
        var count: Long = 0,
        var firstNs: Long = 0,
        var lastNs: Long = 0,
        var minIntervalNs: Long = Long.MAX_VALUE,
        var maxIntervalNs: Long = 0
    ) {
        fun record(timestampNs: Long) {
            if (count == 0L) firstNs = timestampNs
            else {
                val interval = timestampNs - lastNs
                if (interval > 0) {
                    minIntervalNs = minOf(minIntervalNs, interval)
                    maxIntervalNs = maxOf(maxIntervalNs, interval)
                }
            }
            lastNs = timestampNs
            count++
        }

        fun meanHz(): Double? = if (count > 1 && lastNs > firstNs) {
            (count - 1) * 1_000_000_000.0 / (lastNs - firstNs)
        } else null
    }

    private lateinit var sensors: SensorManager
    private lateinit var locations: LocationManager
    private lateinit var workerThread: HandlerThread
    private lateinit var worker: Handler
    private lateinit var mapWorkerThread: HandlerThread
    private lateinit var mapWorker: Handler
    private lateinit var modeTitleView: TextView
    private lateinit var modeDetailView: TextView
    private lateinit var rateValueView: TextView
    private lateinit var rateDetailView: TextView
    private lateinit var gnssValueView: TextView
    private lateinit var gnssDetailView: TextView
    private lateinit var capabilityView: TextView
    private lateinit var vehicleSignalView: TextView
    private lateinit var navigationView: TextView
    private lateinit var mapStatusView: TextView
    private lateinit var sensorsView: TextView
    private lateinit var captureMetaView: TextView
    private lateinit var valuesView: TextView
    private lateinit var startButton: Button
    private lateinit var stopButton: Button
    private lateinit var exportButton: Button
    private val recording = AtomicBoolean(false)
    private val writeLock = Any()
    private val rateLock = Any()
    private val latest = linkedMapOf<String, String>()
    private var writer: BufferedWriter? = null
    private var currentFile: File? = null
    private var profileFile: File? = null
    private var bundleFile: File? = null
    private val rateStats = linkedMapOf<String, RateStats>()
    private val registrationResults = linkedMapOf<String, Boolean>()
    private val signalProcessor = VehicleSignalProcessor()
    private val navigationEngine = PhoneNavigationEngine(outputHz = 10.0)
    @Volatile private var offlineRoadMatcher: OfflineRoadMatcher? = null
    private var latestRotationMatrix: FloatArray? = null
    private var latestAzimuthRad: Double? = null
    private var latestLinearAcceleration = DoubleArray(3)
    private var latestGravity = DoubleArray(3)
    private var latestDeviceGyro = DoubleArray(3)
    private var hasLinearSensor = false
    private var hasGyroSensor = false
    @Volatile private var latestLocation: Location? = null
    @Volatile private var latestNetworkLocation: Location? = null
    @Volatile private var gpsProviderAvailable = false
    @Volatile private var gpsProviderEnabled = false
    @Volatile private var networkProviderEnabled = false
    @Volatile private var locationRequestError: String? = null
    @Volatile private var gpsSatellitesVisible = -1
    @Volatile private var gpsSatellitesUsed = -1
    @Volatile private var gpsRequestStartedNs = 0L
    private val gpsFixCount = AtomicInteger(0)
    private val networkLocationCount = AtomicInteger(0)
    private val gnssStatusCallback = object : GnssStatus.Callback() {
        override fun onSatelliteStatusChanged(status: GnssStatus) {
            var used = 0
            for (index in 0 until status.satelliteCount) if (status.usedInFix(index)) used++
            gpsSatellitesVisible = status.satelliteCount
            gpsSatellitesUsed = used
        }
    }
    private var gnssStatusRegistered = false
    private var gnssStatusWasRegistered = false
    private var firstSensorNs = 0L
    private var lastSensorNs = 0L
    private var accelerometerEvents = 0L
    private var measuredHz = 0.0
    private var captureStartElapsedNs = 0L
    private var captureStartUtcMs = 0L
    @Volatile private var captureWriteError: String? = null
    @Volatile private var csvRowsWritten = 0L
    private var rowsSinceFlush = 0
    private val failureStopQueued = AtomicBoolean(false)
    private val uiHandler = Handler(Looper.getMainLooper())
    private val refreshUi = object : Runnable {
        override fun run() {
            renderStatus()
            if (!isFinishing) uiHandler.postDelayed(this, 250)
        }
    }

    private val sensorTypes = listOf(
        Sensor.TYPE_ACCELEROMETER to "accelerometer",
        Sensor.TYPE_GYROSCOPE to "gyroscope",
        Sensor.TYPE_MAGNETIC_FIELD to "magnetometer",
        Sensor.TYPE_GRAVITY to "gravity",
        Sensor.TYPE_LINEAR_ACCELERATION to "linear_acceleration",
        Sensor.TYPE_ROTATION_VECTOR to "rotation_vector"
    )

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        sensors = getSystemService(Context.SENSOR_SERVICE) as SensorManager
        locations = getSystemService(Context.LOCATION_SERVICE) as LocationManager
        workerThread = HandlerThread("sensor-capture").apply { start() }
        worker = Handler(workerThread.looper)
        mapWorkerThread = HandlerThread("offline-road-map").apply { start() }
        mapWorker = Handler(mapWorkerThread.looper)
        buildUi()
        renderStatus()
        uiHandler.post(refreshUi)
        requestLocationPermissionIfNeeded()
    }

    private fun buildUi() {
        val page = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(dp(20), dp(22), dp(20), dp(28))
            setBackgroundColor(Color.rgb(7, 19, 30))
        }
        val scroll = ScrollView(this).apply {
            setBackgroundColor(Color.rgb(7, 19, 30))
            isFillViewport = true
            clipToPadding = true
            addView(page, android.widget.FrameLayout.LayoutParams(
                android.widget.FrameLayout.LayoutParams.MATCH_PARENT,
                android.widget.FrameLayout.LayoutParams.WRAP_CONTENT
            ))
        }
        scroll.setOnApplyWindowInsetsListener { view, insets ->
            val topInset: Int
            val bottomInset: Int
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
                topInset = insets.getInsets(WindowInsets.Type.statusBars()).top
                bottomInset = insets.getInsets(WindowInsets.Type.navigationBars()).bottom
            } else {
                @Suppress("DEPRECATION")
                run {
                    topInset = insets.systemWindowInsetTop
                    bottomInset = insets.systemWindowInsetBottom
                }
            }
            view.setPadding(0, topInset, 0, bottomInset)
            insets
        }
        page.isFocusableInTouchMode = true
        page.requestFocus()

        val header = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = android.view.Gravity.CENTER_VERTICAL
        }
        val logo = TextView(this).apply {
            text = "SN"
            textSize = 17f
            setTextColor(Color.rgb(7, 25, 34))
            typeface = android.graphics.Typeface.DEFAULT_BOLD
            gravity = android.view.Gravity.CENTER
            background = rounded(Color.rgb(94, 231, 207), 16f)
            layoutParams = LinearLayout.LayoutParams(dp(48), dp(48))
        }
        val titleBlock = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(dp(12), 0, 0, 0)
        }
        titleBlock.addView(label("SeamlessNav", 21f, Color.rgb(240, 247, 250), true))
        titleBlock.addView(label("FIELD SENSOR LAB", 10f, Color.rgb(130, 158, 174), true).apply {
            letterSpacing = 0.12f
            setPadding(0, dp(3), 0, 0)
        })
        header.addView(logo)
        header.addView(titleBlock, LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f))
        header.addView(label("LOGGER MODE", 10f, Color.rgb(94, 231, 207), true).apply {
            setPadding(dp(10), dp(7), dp(10), dp(7))
            background = rounded(Color.rgb(18, 49, 57), 30f)
        })
        page.addView(header)
        page.addView(space(22))

        val hero = panel(Color.rgb(16, 37, 51), Color.rgb(37, 72, 84))
        hero.addView(label("CURRENT SESSION", 10f, Color.rgb(130, 170, 181), true))
        modeTitleView = label("Ready for capture", 25f, Color.rgb(240, 247, 250), true).apply {
            setPadding(0, dp(12), 0, dp(5))
        }
        modeDetailView = label("Start a session to inspect this phone’s live sensor stream.", 13f, Color.rgb(175, 196, 204), false).apply {
            setLineSpacing(dp(3).toFloat(), 1f)
        }
        hero.addView(modeTitleView)
        hero.addView(modeDetailView)
        page.addView(hero, sectionParams())

        val metrics = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }
        val rateCard = panel(Color.rgb(15, 30, 43), Color.rgb(34, 58, 72))
        rateCard.addView(label("IMU INPUT RATE", 10f, Color.rgb(130, 158, 174), true))
        rateValueView = label("— Hz", 23f, Color.rgb(94, 231, 207), true).apply { setPadding(0, dp(10), 0, dp(3)) }
        rateDetailView = label("IMU input request 100 Hz · navigation state target 10 Hz", 11f, Color.rgb(153, 175, 187), false)
        rateCard.addView(rateValueView)
        rateCard.addView(rateDetailView)
        val gpsCard = panel(Color.rgb(15, 30, 43), Color.rgb(34, 58, 72))
        gpsCard.addView(label("GNSS STATUS", 10f, Color.rgb(130, 158, 174), true))
        gnssValueView = label("Waiting", 20f, Color.rgb(255, 203, 119), true).apply { setPadding(0, dp(10), 0, dp(3)) }
        gnssDetailView = label("Enable precise location; test outdoors", 11f, Color.rgb(153, 175, 187), false)
        gpsCard.addView(gnssValueView)
        gpsCard.addView(gnssDetailView)
        metrics.addView(rateCard, LinearLayout.LayoutParams(0, dp(118), 1f).apply { rightMargin = dp(7) })
        metrics.addView(gpsCard, LinearLayout.LayoutParams(0, dp(118), 1f).apply { leftMargin = dp(7) })
        page.addView(metrics, sectionParams())

        val capabilityPanel = panel(Color.rgb(15, 30, 43), Color.rgb(34, 58, 72))
        capabilityPanel.addView(label("DEVICE PROFILE", 10f, Color.rgb(130, 158, 174), true))
        capabilityView = label("Checking sensors…", 14f, Color.rgb(229, 239, 243), true).apply {
            setPadding(0, dp(9), 0, dp(3))
        }
        capabilityPanel.addView(capabilityView)
        page.addView(capabilityPanel, sectionParams())

        val alignmentPanel = panel(Color.rgb(15, 30, 43), Color.rgb(34, 58, 72))
        alignmentPanel.addView(label("ALIGNMENT + MOTION QUALITY", 10f, Color.rgb(130, 158, 174), true))
        vehicleSignalView = label("Start capture to estimate sensor quality and vehicle alignment.", 12f, Color.rgb(197, 214, 220), false).apply {
            setPadding(0, dp(10), 0, 0)
            setLineSpacing(dp(4).toFloat(), 1f)
        }
        alignmentPanel.addView(vehicleSignalView)
        page.addView(alignmentPanel, sectionParams())

        val navigationPanel = panel(Color.rgb(16, 37, 51), Color.rgb(37, 72, 84))
        navigationPanel.addView(label("NAVIGATION STATE OUTPUT · TARGET 10 HZ", 10f, Color.rgb(130, 170, 181), true))
        navigationView = label("IMU input is separate from the 10 Hz navigation output. Start capture and acquire a first GPS fix to anchor position.", 13f, Color.rgb(197, 214, 220), false).apply {
            setPadding(0, dp(10), 0, 0)
            setLineSpacing(dp(4).toFloat(), 1f)
        }
        navigationPanel.addView(navigationView)
        val mapButton = Button(this).apply {
            text = "Load offline road map"
            isAllCaps = false
            setOnClickListener { chooseOfflineRoadMap() }
        }
        navigationPanel.addView(mapButton, LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT, dp(48)
        ).apply { topMargin = dp(12) })
        mapStatusView = label("Offline map not loaded · raw navigation remains available", 11f, Color.rgb(153, 175, 187), false).apply {
            setPadding(0, dp(7), 0, 0)
            setLineSpacing(dp(3).toFloat(), 1f)
        }
        navigationPanel.addView(mapStatusView)
        page.addView(navigationPanel, sectionParams())

        val sensorPanel = panel(Color.rgb(15, 30, 43), Color.rgb(34, 58, 72))
        sensorPanel.addView(label("SENSOR HEALTH", 10f, Color.rgb(130, 158, 174), true))
        sensorsView = label("Start capture to measure per-sensor rates.", 12f, Color.rgb(197, 214, 220), false).apply {
            typeface = android.graphics.Typeface.MONOSPACE
            setPadding(0, dp(11), 0, 0)
            setLineSpacing(dp(7).toFloat(), 1f)
        }
        sensorPanel.addView(sensorsView)
        page.addView(sensorPanel, sectionParams())

        val valuesPanel = panel(Color.rgb(15, 30, 43), Color.rgb(34, 58, 72))
        valuesPanel.addView(label("LIVE SENSOR READINGS", 10f, Color.rgb(130, 158, 174), true))
        valuesView = label("Values will appear when capture starts.", 12f, Color.rgb(197, 214, 220), false).apply {
            typeface = android.graphics.Typeface.MONOSPACE
            setPadding(0, dp(10), 0, 0)
            setLineSpacing(dp(4).toFloat(), 1f)
        }
        valuesPanel.addView(valuesView)
        page.addView(valuesPanel, sectionParams())

        startButton = Button(this).apply {
            text = "Start capture"
            setOnClickListener { startCapture() }
        }
        styleButton(startButton, Color.rgb(94, 231, 207), Color.rgb(7, 25, 34))
        stopButton = Button(this).apply {
            text = "Stop capture"
            isEnabled = false
            setOnClickListener { stopCapture() }
        }
        styleButton(stopButton, Color.rgb(48, 37, 49), Color.rgb(255, 151, 158), Color.rgb(101, 57, 68))
        exportButton = Button(this).apply {
            text = "Export session bundle  ↗"
            isEnabled = false
            setOnClickListener { exportLatest() }
        }
        styleButton(exportButton, Color.rgb(16, 37, 51), Color.rgb(223, 237, 241), Color.rgb(52, 81, 96))
        page.addView(startButton, buttonParams())
        page.addView(stopButton, buttonParams())
        page.addView(exportButton, buttonParams())

        val footer = panel(Color.rgb(30, 33, 34), Color.rgb(77, 73, 57))
        footer.addView(label("PROTOTYPE STATUS", 10f, Color.rgb(255, 203, 119), true))
        footer.addView(label("Phone navigation targets 10 Hz; offline road hypotheses can be loaded. The 200 Hz external-IMU path and under-10% outage benchmark remain unverified.", 12f, Color.rgb(210, 205, 184), false).apply {
            setPadding(0, dp(7), 0, 0)
            setLineSpacing(dp(3).toFloat(), 1f)
        })
        page.addView(footer, sectionParams())
        captureMetaView = label("No capture yet", 10f, Color.rgb(112, 139, 154), false).apply { gravity = android.view.Gravity.CENTER }
        page.addView(captureMetaView, LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT))
        setContentView(scroll)
        scroll.post { scroll.scrollTo(0, 0) }
    }

    private fun dp(value: Int): Int = (value * resources.displayMetrics.density + 0.5f).toInt()

    private fun label(text: String, sizeSp: Float, color: Int, bold: Boolean): TextView = TextView(this).apply {
        this.text = text
        textSize = sizeSp
        setTextColor(color)
        typeface = if (bold) android.graphics.Typeface.DEFAULT_BOLD else android.graphics.Typeface.DEFAULT
        includeFontPadding = false
    }

    private fun rounded(color: Int, radiusDp: Float, strokeColor: Int? = null): GradientDrawable = GradientDrawable().apply {
        setColor(color)
        cornerRadius = dp(radiusDp.toInt()).toFloat()
        strokeColor?.let { setStroke(dp(1), it) }
    }

    private fun panel(color: Int, border: Int): LinearLayout = LinearLayout(this).apply {
        orientation = LinearLayout.VERTICAL
        setPadding(dp(16), dp(16), dp(16), dp(16))
        background = rounded(color, 18f, border)
    }

    private fun space(heightDp: Int): android.view.View = android.view.View(this).apply {
        layoutParams = LinearLayout.LayoutParams(1, dp(heightDp))
    }

    private fun sectionParams(): LinearLayout.LayoutParams = LinearLayout.LayoutParams(
        LinearLayout.LayoutParams.MATCH_PARENT,
        LinearLayout.LayoutParams.WRAP_CONTENT
    ).apply { topMargin = dp(12) }

    private fun buttonParams(): LinearLayout.LayoutParams = LinearLayout.LayoutParams(
        LinearLayout.LayoutParams.MATCH_PARENT,
        dp(54)
    ).apply { topMargin = dp(9) }

    private fun styleButton(button: Button, fill: Int, foreground: Int, border: Int? = null) {
        button.isAllCaps = false
        button.textSize = 15f
        button.typeface = android.graphics.Typeface.DEFAULT_BOLD
        button.setTextColor(foreground)
        button.background = rounded(fill, 16f, border)
        button.elevation = 0f
    }

    private fun requestLocationPermissionIfNeeded() {
        if (checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED &&
            checkSelfPermission(Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            requestPermissions(arrayOf(Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.ACCESS_COARSE_LOCATION), LOCATION_REQUEST)
        }
    }

    override fun onRequestPermissionsResult(requestCode: Int, permissions: Array<out String>, grantResults: IntArray) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == LOCATION_REQUEST && grantResults.any { it == PackageManager.PERMISSION_GRANTED }) {
            Toast.makeText(this, "Location permission granted", Toast.LENGTH_SHORT).show()
        }
    }

    private fun startCapture() {
        if (recording.get()) return
        val stamp = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
        currentFile = File(filesDir, "seamlessnav_$stamp.csv")
        profileFile = File(filesDir, "seamlessnav_${stamp}_profile.json")
        bundleFile = null
        rateStats.clear()
        registrationResults.clear()
        gnssStatusRegistered = false
        gnssStatusWasRegistered = false
        signalProcessor.reset()
        navigationEngine.reset()
        latestRotationMatrix = null
        latestAzimuthRad = null
        latestLinearAcceleration.fill(0.0)
        latestGravity.fill(0.0)
        latestDeviceGyro.fill(0.0)
        hasLinearSensor = false
        hasGyroSensor = false
        captureWriteError = null
        csvRowsWritten = 0L
        rowsSinceFlush = 0
        failureStopQueued.set(false)
        synchronized(latest) { latest.clear() }
        latestLocation = null
        latestNetworkLocation = null
        locationRequestError = null
        gpsRequestStartedNs = 0L
        gpsSatellitesVisible = -1
        gpsSatellitesUsed = -1
        gpsFixCount.set(0)
        networkLocationCount.set(0)
        try {
            synchronized(writeLock) {
                writer = BufferedWriter(OutputStreamWriter(currentFile!!.outputStream(), Charsets.UTF_8))
                writer?.write("sensor,timestamp_elapsed_ns,timestamp_utc_ms,x,y,z,latitude_deg,longitude_deg,horizontal_accuracy_m,speed_mps,bearing_deg\n")
                writer?.flush()
            }
        } catch (e: Exception) {
            writer = null
            captureWriteError = "Could not create capture file: ${e.localizedMessage ?: e.javaClass.simpleName}"
            Toast.makeText(this, captureWriteError, Toast.LENGTH_LONG).show()
            renderStatus()
            return
        }
        firstSensorNs = 0L
        lastSensorNs = 0L
        accelerometerEvents = 0L
        measuredHz = 0.0
        captureStartElapsedNs = android.os.SystemClock.elapsedRealtimeNanos()
        captureStartUtcMs = System.currentTimeMillis()
        recording.set(true)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        sensorTypes.forEach { (type, label) ->
            sensors.getDefaultSensor(type)?.let { sensor ->
                registrationResults[label] = sensors.registerListener(this, sensor, SENSOR_PERIOD_US, 0, worker)
            } ?: run {
                registrationResults[label] = false
            }
        }
        requestGnssUpdates()
        startButton.isEnabled = false
        stopButton.isEnabled = true
        exportButton.isEnabled = false
        Toast.makeText(this, "Capture started. Keep the phone mounted and screen on.", Toast.LENGTH_LONG).show()
    }

    private fun requestGnssUpdates() {
        val hasFine = checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED
        val hasCoarse = checkSelfPermission(Manifest.permission.ACCESS_COARSE_LOCATION) == PackageManager.PERMISSION_GRANTED
        if (!hasFine && !hasCoarse) {
            locationRequestError = "Location permission is not granted"
            requestLocationPermissionIfNeeded()
            return
        }
        gpsRequestStartedNs = android.os.SystemClock.elapsedRealtimeNanos()
        try {
            val providers = locations.allProviders
            gpsProviderAvailable = LocationManager.GPS_PROVIDER in providers
            gpsProviderEnabled = gpsProviderAvailable && locations.isProviderEnabled(LocationManager.GPS_PROVIDER)
            networkProviderEnabled = LocationManager.NETWORK_PROVIDER in providers &&
                locations.isProviderEnabled(LocationManager.NETWORK_PROVIDER)
            if (gpsProviderEnabled) {
                // GNSS receivers commonly provide fixes near 1 Hz. The IMU
                // drives the 10 Hz navigation output between those fixes.
                locations.requestLocationUpdates(LocationManager.GPS_PROVIDER, 1_000L, 0f, this, worker.looper)
            }
            if (networkProviderEnabled) {
                // Diagnostic/warm-start location only; never fuse this as GNSS.
                locations.requestLocationUpdates(LocationManager.NETWORK_PROVIDER, 1_000L, 0f, this, worker.looper)
            }
            if (hasFine) {
                gnssStatusRegistered = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
                    locations.registerGnssStatusCallback(gnssStatusCallback, worker)
                } else {
                    @Suppress("DEPRECATION")
                    locations.registerGnssStatusCallback(gnssStatusCallback)
                }
                gnssStatusWasRegistered = gnssStatusRegistered
            }
        } catch (_: SecurityException) {
            locationRequestError = "Location permission was denied by Android"
            Toast.makeText(this, "Location permission is unavailable; IMU capture will continue", Toast.LENGTH_SHORT).show()
        } catch (e: Exception) {
            locationRequestError = "Could not request location: ${e.localizedMessage ?: e.javaClass.simpleName}"
        }
    }

    override fun onSensorChanged(event: SensorEvent) {
        if (!recording.get()) return
        val name = sensorTypes.firstOrNull { it.first == event.sensor.type }?.second ?: return
        val x = event.values.getOrElse(0) { 0f }.toDouble()
        val y = event.values.getOrElse(1) { 0f }.toDouble()
        val z = event.values.getOrElse(2) { 0f }.toDouble()
        val deviceOrientation = if (name == "rotation_vector" && event.values.size >= 3) {
            val matrix = FloatArray(9)
            val orientation = FloatArray(3)
            SensorManager.getRotationMatrixFromVector(matrix, event.values)
            latestRotationMatrix = matrix
            SensorManager.getOrientation(matrix, orientation)
            latestAzimuthRad = orientation[0].toDouble()
            orientation
        } else null
        when (name) {
            "linear_acceleration" -> { latestLinearAcceleration = doubleArrayOf(x, y, z); hasLinearSensor = true }
            "gravity" -> latestGravity = doubleArrayOf(x, y, z)
            "gyroscope" -> { latestDeviceGyro = doubleArrayOf(x, y, z); hasGyroSensor = true }
        }
        val assessment = signalProcessor.updateSensor(
            name,
            event.timestamp,
            event.values.map { it.toDouble() }.toDoubleArray(),
            deviceOrientation?.get(0)?.toDouble(),
            deviceOrientation?.get(1)?.toDouble(),
            deviceOrientation?.get(2)?.toDouble()
        )
        synchronized(latest) { latest[name] = "${format(x)}, ${format(y)}, ${format(z)} ${unitFor(name)}" }
        synchronized(rateLock) { rateStats.getOrPut(name) { RateStats() }.record(event.timestamp) }
        if (name == "accelerometer") {
            if (!hasLinearSensor) {
                val gravity = if (latestGravity.any { abs(it) > 1.0 }) latestGravity else doubleArrayOf(0.0, 0.0, 9.80665)
                latestLinearAcceleration = DoubleArray(3) { event.values[it].toDouble() - gravity[it] }
            }
            val rotation = latestRotationMatrix
            val location = latestLocation
            val orientationCourse = assessment.alignmentOffsetDeg?.let { offset ->
                ((Math.toDegrees(latestAzimuthRad ?: 0.0) + offset) % 360.0 + 360.0) % 360.0
            }
            val freshCourse = location?.takeIf {
                it.hasBearing() && event.timestamp - it.elapsedRealtimeNanos in 0..2_000_000_000L
            }?.bearing?.toDouble()
            val courseEstimate = orientationCourse ?: freshCourse
            if (rotation != null && hasGyroSensor && courseEstimate != null) {
                val accelEnu = rotateDeviceVector(rotation, latestLinearAcceleration)
                val gyroEnu = rotateDeviceVector(rotation, latestDeviceGyro)
                val courseDeg = courseEstimate
                val heading = Math.toRadians(90.0 - courseDeg)
                val forwardAccel = accelEnu[0] * kotlin.math.cos(heading) + accelEnu[1] * kotlin.math.sin(heading)
                navigationEngine.processImu(event.timestamp, forwardAccel, gyroEnu[2], assessment)?.let { state ->
                    queueNavigationRender(state)
                }
            }
            if (firstSensorNs == 0L) firstSensorNs = event.timestamp
            lastSensorNs = event.timestamp
            accelerometerEvents++
            val elapsed = (lastSensorNs - firstSensorNs) / 1_000_000_000.0
            if (elapsed > 0.25) measuredHz = (accelerometerEvents - 1) / elapsed
        }
        appendRow(name, event.timestamp, x, y, z, null)
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit

    override fun onLocationChanged(location: Location) {
        when (location.provider) {
            LocationManager.GPS_PROVIDER -> {
                latestLocation = location
                gpsFixCount.incrementAndGet()
                signalProcessor.updateGnss(
                    location.elapsedRealtimeNanos,
                    if (location.hasSpeed()) location.speed.toDouble() else null,
                    if (location.hasBearing()) location.bearing.toDouble() else null,
                    location.accuracy.toDouble()
                )
                navigationEngine.updateGnss(location)?.let { state -> queueNavigationRender(state) }
                appendRow("gnss", location.elapsedRealtimeNanos, null, null, null, location)
            }
            LocationManager.NETWORK_PROVIDER -> {
                latestNetworkLocation = location
                networkLocationCount.incrementAndGet()
                // Keep Wi-Fi/cell estimates visible for diagnosis but out of
                // the GNSS/INS update path because their error is not GNSS.
                appendRow("network_location", location.elapsedRealtimeNanos, null, null, null, location)
            }
            else -> appendRow("location_${location.provider ?: "unknown"}", location.elapsedRealtimeNanos, null, null, null, location)
        }
    }

    private fun rotateDeviceVector(matrix: FloatArray, vector: DoubleArray): DoubleArray = doubleArrayOf(
        matrix[0] * vector[0] + matrix[1] * vector[1] + matrix[2] * vector[2],
        matrix[3] * vector[0] + matrix[4] * vector[1] + matrix[5] * vector[2],
        matrix[6] * vector[0] + matrix[7] * vector[1] + matrix[8] * vector[2]
    )

    private fun queueNavigationRender(state: PhoneNavigationState) {
        if (!::mapWorker.isInitialized) {
            uiHandler.post { renderNavigation(state, null) }
            return
        }
        mapWorker.post {
            val match = try { offlineRoadMatcher?.match(state) } catch (_: Exception) { null }
            uiHandler.post { renderNavigation(state, match) }
        }
    }

    private fun renderNavigation(state: PhoneNavigationState, match: OfflineRoadMatcher.Result?) {
        navigationView.text = buildString {
            append("${state.mode.replace('_', ' ')} · ${format(state.speedMps)} m/s (${format(state.speedMps * 3.6)} km/h)")
            append("\n${format(state.latitudeDeg)}°, ${format(state.longitudeDeg)}° · heading ${format(state.headingDeg)}°")
            append("\nHorizontal uncertainty ±${format(state.horizontalSigmaM)} m")
            append("\nGNSS update ${if (state.acceptedGnss) "accepted" else "not accepted"} · phone output target 10 Hz")
            if (match != null) {
                if (match.accepted) append("\nMap hypothesis ${match.latitudeDeg?.let(::format)}°, ${match.longitudeDeg?.let(::format)}° · confidence ${format(match.confidence)}")
                else append("\nMap matcher abstained · confidence ${format(match.confidence)} · ${match.reason}")
            }
        }
        if (match?.accepted == true) {
            mapStatusView.text = "Matched road ${match.roadId} · ${format(match.distanceM ?: 0.0)} m from estimate · confidence ${format(match.confidence)}\n${offlineRoadMatcher?.attribution ?: "OpenStreetMap contributors"}"
            mapStatusView.setTextColor(Color.rgb(94, 231, 207))
        } else if (offlineRoadMatcher != null && match != null) {
            mapStatusView.text = "No confident road match · ${match.reason}; raw GNSS/INS estimate retained."
            mapStatusView.setTextColor(Color.rgb(255, 203, 119))
        }
    }

    private fun chooseOfflineRoadMap() {
        val intent = Intent(Intent.ACTION_OPEN_DOCUMENT).apply {
            addCategory(Intent.CATEGORY_OPENABLE)
            type = "application/json"
        }
        @Suppress("DEPRECATION")
        startActivityForResult(intent, MAP_REQUEST)
    }

    private fun loadOfflineRoadMap(uri: android.net.Uri) {
        mapStatusView.text = "Copying and indexing offline road data…"
        mapStatusView.setTextColor(Color.rgb(153, 175, 187))
        mapWorker.post {
            try {
                val target = File(filesDir, "offline-roads-v1.json")
                val temporary = File(filesDir, "offline-roads-v1.json.tmp")
                contentResolver.openInputStream(uri)?.use { input -> FileOutputStream(temporary).use { output -> input.copyTo(output) } }
                    ?: throw IllegalStateException("Could not open the selected road database")
                if (target.exists() && !target.delete()) throw IllegalStateException("Could not replace previous offline map")
                if (!temporary.renameTo(target)) throw IllegalStateException("Could not save offline map on the phone")
                val matcher = OfflineRoadMatcher.load(target)
                offlineRoadMatcher = matcher
                uiHandler.post {
                    mapStatusView.text = "Offline map loaded · ${target.length() / 1024} KB · ${matcher.attribution}"
                    mapStatusView.setTextColor(Color.rgb(94, 231, 207))
                    navigationEngine.snapshot()?.let(::queueNavigationRender)
                    Toast.makeText(this, "Offline road map loaded", Toast.LENGTH_SHORT).show()
                }
            } catch (error: Exception) {
                uiHandler.post {
                    mapStatusView.text = "Map load failed · ${error.localizedMessage ?: error.javaClass.simpleName}"
                    mapStatusView.setTextColor(Color.rgb(255, 143, 153))
                }
            }
        }
    }

    @Deprecated("Deprecated by Android; retained for older API levels")
    override fun onStatusChanged(provider: String?, status: Int, extras: Bundle?) = Unit
    override fun onProviderEnabled(provider: String) { refreshProviderStatus() }
    override fun onProviderDisabled(provider: String) { refreshProviderStatus() }

    private fun refreshProviderStatus() {
        gpsProviderEnabled = try { locations.isProviderEnabled(LocationManager.GPS_PROVIDER) } catch (_: Exception) { false }
        networkProviderEnabled = try { locations.isProviderEnabled(LocationManager.NETWORK_PROVIDER) } catch (_: Exception) { false }
    }

    private fun appendRow(name: String, elapsedNs: Long, x: Double?, y: Double?, z: Double?, fix: Location?) {
        if (!recording.get()) return
        val utcMs = captureStartUtcMs + ((elapsedNs - captureStartElapsedNs) / 1_000_000L)
        val fields = listOf(
            name, elapsedNs.toString(), utcMs.toString(), csv(x), csv(y), csv(z),
            csv(fix?.latitude), csv(fix?.longitude), csv(fix?.accuracy?.toDouble()),
            csv(if (fix?.hasSpeed() == true) fix.speed.toDouble() else null),
            csv(if (fix?.hasBearing() == true) fix.bearing.toDouble() else null)
        ).joinToString(",")
        synchronized(writeLock) {
            try {
                val activeWriter = writer ?: throw IllegalStateException("Capture file writer is unavailable")
                activeWriter.write(fields)
                activeWriter.newLine()
                csvRowsWritten++
                rowsSinceFlush++
                if (rowsSinceFlush >= 128) {
                    activeWriter.flush()
                    rowsSinceFlush = 0
                }
            } catch (e: Exception) {
                captureWriteError = "Capture stopped after a CSV write error: ${e.localizedMessage ?: e.javaClass.simpleName}"
                if (failureStopQueued.compareAndSet(false, true)) {
                    uiHandler.post {
                        stopCapture()
                        Toast.makeText(this, captureWriteError ?: "Capture file write failed", Toast.LENGTH_LONG).show()
                    }
                }
            }
        }
    }

    private fun stopCapture() {
        if (!recording.getAndSet(false)) return
        sensors.unregisterListener(this)
        try { locations.removeUpdates(this) } catch (_: SecurityException) { }
        if (gnssStatusRegistered) {
            try { locations.unregisterGnssStatusCallback(gnssStatusCallback) } catch (_: Exception) { }
            gnssStatusRegistered = false
        }
        synchronized(writeLock) {
            try { writer?.flush(); writer?.close() } catch (e: Exception) {
                if (captureWriteError == null) captureWriteError = "Could not finalize capture file: ${e.localizedMessage ?: e.javaClass.simpleName}"
            }
            writer = null
        }
        window.clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        try { writeSessionProfile() } catch (e: Exception) {
            if (captureWriteError == null) captureWriteError = "Could not save sensor profile: ${e.localizedMessage ?: e.javaClass.simpleName}"
        }
        startButton.isEnabled = true
        stopButton.isEnabled = false
        exportButton.isEnabled = currentFile?.exists() == true
        Toast.makeText(this, "Capture saved privately on this device", Toast.LENGTH_SHORT).show()
    }

    private fun exportLatest() {
        val file = currentFile ?: return
        val profile = profileFile ?: return
        if (!file.exists() || !profile.exists()) return
        val bundle = File(cacheDir, "${file.nameWithoutExtension}_bundle.zip")
        try {
            ZipOutputStream(FileOutputStream(bundle)).use { zip ->
                listOf(file, profile).forEach { source ->
                    zip.putNextEntry(ZipEntry(source.name))
                    FileInputStream(source).use { input -> input.copyTo(zip) }
                    zip.closeEntry()
                }
            }
            bundleFile = bundle
        } catch (e: Exception) {
            Toast.makeText(this, "Could not package capture: ${e.localizedMessage}", Toast.LENGTH_LONG).show()
            return
        }
        val intent = Intent(Intent.ACTION_CREATE_DOCUMENT).apply {
            addCategory(Intent.CATEGORY_OPENABLE)
            type = "application/zip"
            putExtra(Intent.EXTRA_TITLE, bundle.name)
        }
        @Suppress("DEPRECATION")
        startActivityForResult(intent, EXPORT_REQUEST)
    }

    @Deprecated("Deprecated by Android; retained for platform Activity compatibility")
    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?) {
        super.onActivityResult(requestCode, resultCode, data)
        if (requestCode == MAP_REQUEST && resultCode == RESULT_OK) {
            data?.data?.let(::loadOfflineRoadMap)
            return
        }
        if (requestCode != EXPORT_REQUEST || resultCode != RESULT_OK) return
        val uri = data?.data ?: return
        val source = bundleFile ?: return
        try {
            contentResolver.openOutputStream(uri)?.use { output -> FileInputStream(source).use { input -> input.copyTo(output) } }
            Toast.makeText(this, "CSV and sensor profile exported", Toast.LENGTH_SHORT).show()
        } catch (e: Exception) {
            Toast.makeText(this, "Export failed: ${e.localizedMessage}", Toast.LENGTH_LONG).show()
        }
    }

    private fun renderStatus() {
        val accelerometer = sensors.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        val gyroscope = sensors.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        val capabilityProfile = capabilityProfile(accelerometer != null, gyroscope != null)
        val rates = synchronized(rateLock) { rateStats.mapValues { it.value.meanHz() } }
        val active = recording.get()
        modeTitleView.text = when {
            active -> "Recording session"
            csvRowsWritten > 0L -> "Capture complete"
            else -> "Ready for capture"
        }
        modeTitleView.setTextColor(if (active) Color.rgb(94, 231, 207) else Color.rgb(240, 247, 250))
        modeDetailView.text = when {
            captureWriteError != null -> captureWriteError
            active -> "Raw sensor capture is active. Keep the phone mounted and this screen open."
            csvRowsWritten > 0L -> "Session saved on this phone. Export the bundle to review its sensor profile and CSV."
            else -> "Start a session to inspect this phone’s live sensor stream and GNSS availability."
        }
        modeDetailView.setTextColor(if (captureWriteError != null) Color.rgb(255, 151, 158) else Color.rgb(175, 196, 204))

        val accelHz = rates["accelerometer"] ?: measuredHz.takeIf { it > 0.0 }
        val gyroHz = rates["gyroscope"]
        rateValueView.text = accelHz?.let { "${format(it)} Hz" } ?: "— Hz"
        rateDetailView.text = when {
            gyroHz != null -> "IMU input · accel + gyro ${format(gyroHz)} Hz; 100 Hz requested · nav output target 10 Hz"
            accelerometer == null -> "Accelerometer unavailable"
            else -> "IMU input · request 100 Hz; navigation output target 10 Hz"
        }

        val fix = latestLocation
        val networkFix = latestNetworkLocation
        val hasLocationPermission = checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED ||
            checkSelfPermission(Manifest.permission.ACCESS_COARSE_LOCATION) == PackageManager.PERMISSION_GRANTED
        gpsProviderAvailable = try { LocationManager.GPS_PROVIDER in locations.allProviders } catch (_: Exception) { false }
        gpsProviderEnabled = try { gpsProviderAvailable && locations.isProviderEnabled(LocationManager.GPS_PROVIDER) } catch (_: Exception) { false }
        networkProviderEnabled = try {
            LocationManager.NETWORK_PROVIDER in locations.allProviders && locations.isProviderEnabled(LocationManager.NETWORK_PROVIDER)
        } catch (_: Exception) { false }
        val fixAgeSeconds = fix?.let { maxOf(0L, android.os.SystemClock.elapsedRealtimeNanos() - it.elapsedRealtimeNanos) / 1e9 }
        if (fix != null && recording.get() && (fixAgeSeconds ?: Double.POSITIVE_INFINITY) <= 5.0) {
            gnssValueView.text = "GPS FIX RECEIVED"
            gnssValueView.setTextColor(Color.rgb(94, 231, 207))
            gnssDetailView.text = "±${format(fix.accuracy.toDouble())} m GPS fix · " +
                (if (fix.hasSpeed()) "${format(fix.speed.toDouble())} m/s" else "speed unavailable")
        } else if (fix != null) {
            gnssValueView.text = if (recording.get()) "GPS SIGNAL STALE" else "LAST GPS FIX"
            gnssValueView.setTextColor(Color.rgb(255, 203, 119))
            gnssDetailView.text = "Last fix ${format(fixAgeSeconds ?: 0.0)} s ago · ±${format(fix.accuracy.toDouble())} m"
        } else if (networkFix != null) {
            gnssValueView.text = "NO GPS FIX"
            gnssValueView.setTextColor(Color.rgb(255, 203, 119))
            gnssDetailView.text = "Network estimate ±${format(networkFix.accuracy.toDouble())} m only; not fused as GNSS. Try open sky."
        } else {
            gnssValueView.text = when {
                !hasLocationPermission -> "Permission needed"
                !gpsProviderAvailable -> "GPS unavailable"
                !gpsProviderEnabled -> "Location is off"
                locationRequestError != null -> "Location request failed"
                recording.get() -> "Searching for GPS"
                else -> "No GPS fix yet"
            }
            gnssValueView.setTextColor(Color.rgb(255, 203, 119))
            gnssDetailView.text = when {
                !hasLocationPermission -> "Grant precise location permission"
                !gpsProviderAvailable -> "No GPS provider is exposed by Android on this device"
                !gpsProviderEnabled -> "Turn on Location/GPS in Quick Settings, then retry"
                locationRequestError != null -> locationRequestError
                recording.get() -> {
                    val elapsed = if (gpsRequestStartedNs > 0L) maxOf(0L, (android.os.SystemClock.elapsedRealtimeNanos() - gpsRequestStartedNs) / 1_000_000_000L) else 0L
                    val satellites = if (gpsSatellitesVisible >= 0) "$gpsSatellitesUsed/$gpsSatellitesVisible satellites used · " else ""
                    "${satellites}searching ${elapsed}s · keep the phone still outdoors with open sky"
                }
                else -> "Start capture to request a live GPS fix"
            }
        }

        capabilityView.text = when (capabilityProfile) {
            "six_axis_accel_gyro" -> "Six-axis IMU available · accelerometer + gyroscope"
            "reduced_accel_only" -> "Reduced inertial mode · no gyroscope detected"
            else -> "GNSS-only profile · inertial dead reckoning unavailable"
        }
        val signal = signalProcessor.snapshot()
        vehicleSignalView.text = buildString {
            append(signal.event.replace('_', ' '))
            append(" · quality ${format(signal.quality)}")
            append("\nAlignment: ")
            append(if (signal.alignmentOffsetDeg == null) "awaiting straight GNSS-aided motion" else "${format(signal.alignmentOffsetDeg!!)}° offset · ${format(signal.alignmentConfidence * 100.0)}% confidence")
            if (signal.phonePitchDeg != null && signal.phoneRollDeg != null) {
                append("\nPhone tilt: ${format(signal.phonePitchDeg)}° pitch · ${format(signal.phoneRollDeg)}° roll vs gravity")
            }
            append("\nGyro bias: ${signal.gyroBiasRadps.joinToString(", ") { format(it) }} rad/s")
            append("\n${signal.detail}")
        }
        val navState = navigationEngine.snapshot()
        if (navState != null) queueNavigationRender(navState)
        else if (latestLocation == null) {
            navigationView.text = when {
                recording.get() && lastSensorNs > 0L -> "IMU input is active; waiting for the first GPS fix to anchor position. IMU input rate and navigation output rate are different."
                !recording.get() && csvRowsWritten > 0L -> "Session stopped. IMU rows were recorded, but no GPS fix was received, so no absolute navigation state was available."
                else -> "Waiting for IMU input and the first GPS fix. IMU request is 100 Hz; navigation state target is 10 Hz."
            }
        }
        val sensorLines = sensorTypes.map { (type, name) ->
            val sensor = sensors.getDefaultSensor(type)
            val status = when {
                sensor == null -> "NOT PRESENT"
                active && registrationResults[name] == false -> "LISTENER FAILED"
                rates[name] != null -> "${format(rates[name]!!)} Hz"
                else -> "READY"
            }
            "${name.uppercase(Locale.US).padEnd(22)} $status"
        }
        sensorsView.text = sensorLines.joinToString("\n")
        val values = synchronized(latest) { latest.toMap() }
        valuesView.text = if (values.isEmpty()) "Waiting for the first sensor event…" else
            values.entries.joinToString("\n") { "${it.key.uppercase(Locale.US)}\n  ${it.value}" }
        captureMetaView.text = buildString {
            append("${csvRowsWritten} ROWS SAVED")
            currentFile?.name?.let { append("  ·  $it") }
            if (active) append("  ·  LIVE")
        }
        startButton.isEnabled = !active
        stopButton.isEnabled = active
        exportButton.isEnabled = !active && currentFile?.exists() == true && profileFile?.exists() == true
    }

    private fun unitFor(name: String) = when (name) {
        "accelerometer", "linear_acceleration", "gravity" -> "m/s²"
        "gyroscope" -> "rad/s"
        "magnetometer" -> "µT"
        else -> "sensor units"
    }

    private fun format(value: Double) = String.format(Locale.US, "%.3f", value)
    private fun csv(value: Double?) = value?.let { format(it) } ?: ""

    private fun capabilityProfile(hasAccelerometer: Boolean, hasGyroscope: Boolean) = when {
        hasAccelerometer && hasGyroscope -> "six_axis_accel_gyro"
        hasAccelerometer -> "reduced_accel_only"
        else -> "gnss_only_no_accelerometer"
    }

    private fun writeSessionProfile() {
        val output = profileFile ?: return
        val nowElapsedNs = android.os.SystemClock.elapsedRealtimeNanos()
        val available = sensors.getSensorList(Sensor.TYPE_ALL)
        val inventory = JSONArray()
        available.forEach { sensor ->
            inventory.put(JSONObject()
                .put("type_id", sensor.type)
                .put("type", sensor.stringType)
                .put("name", sensor.name)
                .put("vendor", sensor.vendor)
                .put("version", sensor.version)
                .put("resolution", sensor.resolution.toDouble())
                .put("maximum_range", sensor.maximumRange.toDouble())
                .put("minimum_delay_us", sensor.minDelay)
                .put("maximum_delay_us", sensor.maxDelay)
                .put("power_ma", sensor.power.toDouble())
                .put("reporting_mode", sensor.reportingMode)
                .put("wake_up", sensor.isWakeUpSensor))
        }
        val rates = JSONObject()
        synchronized(rateLock) {
            rateStats.forEach { (name, stats) ->
                val rate = JSONObject()
                    .put("samples", stats.count)
                    .put("mean_hz", stats.meanHz())
                    .put("first_timestamp_elapsed_ns", stats.firstNs)
                    .put("last_timestamp_elapsed_ns", stats.lastNs)
                if (stats.minIntervalNs != Long.MAX_VALUE) rate.put("min_interval_ns", stats.minIntervalNs)
                if (stats.maxIntervalNs > 0) rate.put("max_interval_ns", stats.maxIntervalNs)
                rates.put(name, rate)
            }
        }
        val registered = JSONObject()
        registrationResults.forEach { (name, success) -> registered.put(name, success) }
        val hasAccelerometer = sensors.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) != null
        val hasGyroscope = sensors.getDefaultSensor(Sensor.TYPE_GYROSCOPE) != null
        val profile = JSONObject()
            .put("schema_version", 1)
            .put("capture_file", currentFile?.name)
            .put("device_manufacturer", Build.MANUFACTURER)
            .put("device_brand", Build.BRAND)
            .put("device_model", Build.MODEL)
            .put("device", Build.DEVICE)
            .put("android_release", Build.VERSION.RELEASE)
            .put("android_sdk", Build.VERSION.SDK_INT)
            .put("capability_profile", capabilityProfile(hasAccelerometer, hasGyroscope))
            .put("requested_sensor_period_us", SENSOR_PERIOD_US)
            .put("timestamp_clock", "elapsed_realtime_nanos; monotonic")
            .put("sensor_axis_frame", "Android device sensor frame")
            .put("sensor_units", JSONObject()
                .put("accelerometer", "m/s^2")
                .put("linear_acceleration", "m/s^2")
                .put("gravity", "m/s^2")
                .put("gyroscope", "rad/s")
                .put("magnetometer", "microtesla")
                .put("rotation_vector", "Android rotation-vector components"))
            .put("capture_start_utc_ms", captureStartUtcMs)
            .put("capture_end_utc_ms", captureStartUtcMs + ((nowElapsedNs - captureStartElapsedNs) / 1_000_000L))
            .put("csv_rows_written", csvRowsWritten)
            .put("capture_write_error", captureWriteError)
            .put("sensor_registration_succeeded", registered)
            .put("observed_sensor_rates", rates)
            .put("location_diagnostics", JSONObject()
                .put("fine_permission_granted", checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED)
                .put("coarse_permission_granted", checkSelfPermission(Manifest.permission.ACCESS_COARSE_LOCATION) == PackageManager.PERMISSION_GRANTED)
                .put("gps_provider_available", gpsProviderAvailable)
                .put("gps_provider_enabled", gpsProviderEnabled)
                .put("network_provider_enabled", networkProviderEnabled)
                .put("request_error", locationRequestError)
                .put("gps_status_callback_registered", gnssStatusWasRegistered)
                .put("gps_satellites_visible_last", gpsSatellitesVisible)
                .put("gps_satellites_used_last", gpsSatellitesUsed)
                .put("gps_location_callbacks", gpsFixCount.get())
                .put("network_location_callbacks", networkLocationCount.get())
                .put("latest_gps_accuracy_m", latestLocation?.accuracy?.toDouble())
                .put("latest_network_accuracy_m", latestNetworkLocation?.accuracy?.toDouble()))
            .put("sensor_inventory", inventory)
        output.writeText(profile.toString(2), Charsets.UTF_8)
    }

    override fun onPause() {
        super.onPause()
        if (recording.get()) stopCapture()
    }

    override fun onDestroy() {
        uiHandler.removeCallbacks(refreshUi)
        stopCapture()
        workerThread.quitSafely()
        mapWorkerThread.quitSafely()
        super.onDestroy()
    }

    companion object {
        private const val LOCATION_REQUEST = 42
        private const val EXPORT_REQUEST = 43
        private const val MAP_REQUEST = 44
        private const val SENSOR_PERIOD_US = 10_000 // Request 100 Hz; show measured delivered rate instead of assuming it.
    }
}
