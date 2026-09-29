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
import android.widget.EditText
import android.widget.LinearLayout
import android.widget.ImageView
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
import kotlin.math.sqrt

/** First phone milestone: inspect sensors and capture synchronized, raw observations. */
class MainActivity : Activity(), SensorEventListener, LocationListener {
    private data class RateStats(
        var count: Long = 0,
        var firstNs: Long = 0,
        var lastNs: Long = 0,
        var minIntervalNs: Long = Long.MAX_VALUE,
        var maxIntervalNs: Long = 0,
        var intervalSumNs: Double = 0.0,
        var intervalSquaredSumNs2: Double = 0.0
    ) {
        fun record(timestampNs: Long) {
            if (count == 0L) firstNs = timestampNs
            else {
                val interval = timestampNs - lastNs
                if (interval > 0) {
                    minIntervalNs = minOf(minIntervalNs, interval)
                    maxIntervalNs = maxOf(maxIntervalNs, interval)
                    intervalSumNs += interval.toDouble()
                    intervalSquaredSumNs2 += interval.toDouble() * interval.toDouble()
                }
            }
            lastNs = timestampNs
            count++
        }

        fun meanHz(): Double? = if (count > 1 && lastNs > firstNs) {
            (count - 1) * 1_000_000_000.0 / (lastNs - firstNs)
        } else null

        fun meanIntervalNs(): Double? = if (count > 1) intervalSumNs / (count - 1) else null

        fun jitterRmsNs(): Double? = meanIntervalNs()?.let { mean ->
            sqrt((intervalSquaredSumNs2 / (count - 1) - mean * mean).coerceAtLeast(0.0))
        }

        fun clear() {
            count = 0; firstNs = 0; lastNs = 0
            minIntervalNs = Long.MAX_VALUE; maxIntervalNs = 0
            intervalSumNs = 0.0; intervalSquaredSumNs2 = 0.0
        }
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
    private lateinit var navigationStatusView: TextView
    private lateinit var speedStatView: TextView
    private lateinit var turnStatView: TextView
    private lateinit var accuracyStatView: TextView
    private lateinit var driftStatView: TextView
    private lateinit var driftNoteView: TextView
    private lateinit var gnssDiagnosticsView: TextView
    private lateinit var gnssDetailsButton: Button
    private lateinit var sensorDetailsButton: Button
    private lateinit var matchStatView: TextView
    private lateinit var cadenceStatView: TextView
    private lateinit var mapHeaderStatusView: TextView
    private lateinit var navigationSummaryPanel: LinearLayout
    private lateinit var originInput: EditText
    private lateinit var destinationInput: EditText
    private lateinit var routeSearchPanel: LinearLayout
    private lateinit var placeResultsPanel: LinearLayout
    private lateinit var routeSearchHelpView: TextView
    private lateinit var destinationSearchButton: Button
    private lateinit var destinationPickCancelButton: Button
    private lateinit var mapStatusView: TextView
    private lateinit var routeStatusView: TextView
    private lateinit var offlineMapView: OfflineRoadMapView
    private lateinit var sensorsView: TextView
    private lateinit var captureMetaView: TextView
    private lateinit var valuesView: TextView
    private lateinit var benchmarkPhoneEvidenceView: ImageView
    private lateinit var benchmarkPhoneEvidenceNote: TextView
    private var benchmarkDemoRunnable: Runnable? = null
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
    private val navigationOutputStats = RateStats()
    private val sessionNavigationOutputStats = RateStats()
    private var latestMapMatch: OfflineRoadMatcher.Result? = null
    private val registrationResults = linkedMapOf<String, Boolean>()
    private val signalProcessor = VehicleSignalProcessor()
    private val navigationEngine = PhoneNavigationEngine(outputHz = 10.0)
    private val frameCalibrator = VehicleFrameCalibrator()
    private var speedModel: PortableSpeedModel? = null
    private var modelStatus = "Model not loaded"
    private var filteredForward: Double? = null
    private var lastForwardNs = 0L
    private var lastQueuedNavigationNs = 0L
    private var navigationFile: File? = null
    private var navigationWriter: BufferedWriter? = null
    private var roadDownloadBusy = false
    private var lastRoadDownloadMs = 0L
    @Volatile private var pendingDestination: PlaceSearchClient.Place? = null
    @Volatile private var routeRequestInFlight = false
    @Volatile private var pendingAutoRoute = false
    @Volatile private var offlineRoadMatcher: OfflineRoadMatcher? = null
    private var latestRotationMatrix: FloatArray? = null
    private var latestAzimuthRad: Double? = null
    private var latestLinearAcceleration = DoubleArray(3)
    private var latestGravity = DoubleArray(3)
    private var hasGravitySensor = false
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
    private var autoStartPendingPermission = false
    private var captureStartElapsedNs = 0L
    private var captureStartUtcMs = 0L
    private var lastNavigationCsvTimestampNs = 0L
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
        try {
            speedModel = PortableSpeedModel(JSONObject(assets.open("speed-model-v1.json").bufferedReader().use { it.readText() }))
            modelStatus = "On-device learned speed + uncertainty · evaluation mode"
        } catch (e: Exception) { modelStatus = "AI model unavailable: ${e.localizedMessage}" }
        buildUi()
        restoreOfflineRoadMap()
        renderStatus()
        uiHandler.post(refreshUi)
        val hasLocationPermission = checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED ||
            checkSelfPermission(Manifest.permission.ACCESS_COARSE_LOCATION) == PackageManager.PERMISSION_GRANTED
        if (hasLocationPermission) startCapture() else {
            autoStartPendingPermission = true
            requestLocationPermissionIfNeeded()
        }
    }

    private fun buildUi() {
        window.statusBarColor = Color.rgb(244, 246, 246)
        window.navigationBarColor = Color.rgb(244, 246, 246)
        @Suppress("DEPRECATION")
        run { window.decorView.systemUiVisibility = android.view.View.SYSTEM_UI_FLAG_LIGHT_STATUS_BAR or android.view.View.SYSTEM_UI_FLAG_LIGHT_NAVIGATION_BAR }
        val page = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(0, 0, 0, dp(24))
            setBackgroundColor(Color.rgb(238, 241, 237))
        }
        val mapTabContent = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setBackgroundColor(Color.rgb(238, 241, 237))
        }
        val sessionTabContent = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(dp(16), dp(12), dp(16), dp(16))
            setBackgroundColor(Color.rgb(238, 241, 237))
        }
        val scroll = ScrollView(this).apply {
            setBackgroundColor(Color.rgb(238, 241, 237))
            isFillViewport = true
            clipToPadding = true
            addView(page, android.widget.FrameLayout.LayoutParams(
                android.widget.FrameLayout.LayoutParams.MATCH_PARENT,
                android.widget.FrameLayout.LayoutParams.WRAP_CONTENT
            ))
        }
        scroll.setOnApplyWindowInsetsListener { _, insets -> insets }
        page.isFocusableInTouchMode = true
        page.requestFocus()

        val header = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = android.view.Gravity.CENTER_VERTICAL
        }
        val logo = ImageView(this).apply {
            setImageResource(R.drawable.navis_logo)
            contentDescription = "NAVIS logo"
            scaleType = ImageView.ScaleType.FIT_CENTER
            layoutParams = LinearLayout.LayoutParams(dp(56), dp(56))
        }
        val titleBlock = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(dp(12), 0, 0, 0)
        }
        titleBlock.addView(label("NAVIS", 21f, Color.rgb(22, 35, 47), true))
        titleBlock.addView(label("NAVIGATION AI · GNSS + INERTIAL", 9f, Color.rgb(102, 119, 128), true).apply {
            letterSpacing = 0.12f
            setPadding(0, dp(3), 0, 0)
        })
        header.addView(logo)
        header.addView(titleBlock, LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f))
        mapHeaderStatusView = label("GNSS\nWAITING", 8f, Color.rgb(17, 122, 98), true).apply {
            gravity = android.view.Gravity.CENTER
            setPadding(dp(10), dp(6), dp(10), dp(6))
            background = rounded(Color.rgb(231, 249, 243), 22f, Color.rgb(90, 198, 169))
        }
        header.addView(mapHeaderStatusView)
        page.addView(header)
        page.addView(space(14))

        val hero = panel(Color.WHITE, Color.rgb(220, 228, 231))
        hero.addView(label("CURRENT SESSION", 10f, Color.rgb(94, 111, 120), true))
        modeTitleView = label("Starting navigation", 25f, Color.rgb(23, 43, 54), true).apply {
            setPadding(0, dp(12), 0, dp(5))
        }
        modeDetailView = label("Session starts automatically after the Android location permission check. Capture stays on this phone.", 13f, Color.rgb(91, 108, 118), false).apply {
            setLineSpacing(dp(3).toFloat(), 1f)
        }
        hero.addView(modeTitleView)
        hero.addView(modeDetailView)
        page.addView(label("SYSTEM DIAGNOSTICS", 18f, Color.rgb(23, 43, 54), true).apply {
            setPadding(dp(16), dp(14), dp(16), dp(2))
        })
        page.addView(label("GNSS, inertial sensors and filter health", 11f, Color.rgb(91, 108, 118), false).apply {
            setPadding(dp(16), 0, dp(16), dp(4))
        })
        page.addView(hero, sectionParams())

        val metrics = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }
        val rateCard = panel(Color.WHITE, Color.rgb(220, 228, 231))
        rateCard.minimumHeight = dp(126)
        rateCard.addView(label("IMU INPUT RATE", 10f, Color.rgb(94, 111, 120), true))
        rateValueView = label("— Hz", 23f, Color.rgb(19, 145, 124), true).apply { setPadding(0, dp(10), 0, dp(3)) }
        rateDetailView = label("IMU input request 100 Hz · navigation state target 10 Hz", 11f, Color.rgb(91, 108, 118), false)
        rateCard.addView(rateValueView)
        rateCard.addView(rateDetailView)
        val gpsCard = panel(Color.WHITE, Color.rgb(220, 228, 231))
        gpsCard.minimumHeight = dp(126)
        gpsCard.addView(label("GNSS STATUS", 10f, Color.rgb(94, 111, 120), true))
        gnssValueView = label("Waiting", 20f, Color.rgb(174, 111, 21), true).apply { setPadding(0, dp(10), 0, dp(3)) }
        gnssDetailView = label("Enable precise location; test outdoors", 11f, Color.rgb(91, 108, 118), false)
        gpsCard.addView(gnssValueView)
        gpsCard.addView(gnssDetailView)
        metrics.addView(rateCard, LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f).apply {
            rightMargin = dp(7)
        })
        metrics.addView(gpsCard, LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f).apply {
            leftMargin = dp(7)
        })
        page.addView(metrics, sectionParams())

        val capabilityPanel = panel(Color.WHITE, Color.rgb(220, 228, 231))
        capabilityPanel.addView(label("DEVICE PROFILE", 10f, Color.rgb(94, 111, 120), true))
        capabilityView = label("Checking sensors…", 14f, Color.rgb(37, 54, 65), true).apply {
            setPadding(0, dp(9), 0, dp(3))
        }
        capabilityPanel.addView(capabilityView)
        page.addView(capabilityPanel, sectionParams())

        val alignmentPanel = panel(Color.WHITE, Color.rgb(220, 228, 231))
        alignmentPanel.addView(label("ALIGNMENT + MOTION QUALITY", 10f, Color.rgb(94, 111, 120), true))
        vehicleSignalView = label("Start capture to estimate sensor quality and vehicle alignment.", 12f, Color.rgb(37, 54, 65), false).apply {
            setPadding(0, dp(10), 0, 0)
            setLineSpacing(dp(4).toFloat(), 1f)
        }
        alignmentPanel.addView(vehicleSignalView)
        page.addView(alignmentPanel, sectionParams())

        val navigationPanel = panel(Color.rgb(238, 241, 237), Color.rgb(238, 241, 237))
        navigationPanel.background = null
        navigationPanel.setPadding(0, 0, 0, 0)
        navigationPanel.addView(label("LIVE MAP · NAVIGATION TARGET 10 HZ", 10f, Color.rgb(130, 170, 181), true).apply { visibility = android.view.View.GONE })
        navigationView = label("IMU input is separate from the 10 Hz navigation output. Start capture and acquire a first GPS fix to anchor position.", 13f, Color.rgb(197, 214, 220), false).apply {
            setPadding(0, dp(10), 0, 0)
            setLineSpacing(dp(4).toFloat(), 1f)
        }
        offlineMapView = OfflineRoadMapView(this).apply {
            setBackgroundColor(Color.rgb(7, 21, 33))
            setSearchOverlayVisible(true)
            contentDescription = "OpenStreetMap basemap centered on phone location, with navigation and map-matched position"
            onDestinationTap = { latitude, longitude ->
                destinationPickCancelButton.visibility = android.view.View.GONE
                offlineMapView.cancelDestinationPicking()
                planMapPinRoute(latitude, longitude)
            }
        }
        val mapStage = android.widget.FrameLayout(this)
        mapStage.setBackgroundColor(Color.rgb(238, 241, 237))
        mapStage.addView(offlineMapView, android.widget.FrameLayout.LayoutParams(
            android.widget.FrameLayout.LayoutParams.MATCH_PARENT, android.widget.FrameLayout.LayoutParams.MATCH_PARENT
        ).apply { topMargin = dp(62) })
        page.removeView(header)
        header.setPadding(dp(12), dp(3), dp(12), 0)
        mapStage.addView(header, android.widget.FrameLayout.LayoutParams(
            android.widget.FrameLayout.LayoutParams.MATCH_PARENT, dp(56), android.view.Gravity.TOP
        ))
        routeSearchPanel = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(dp(10), dp(9), dp(10), dp(9))
            background = rounded(Color.rgb(250, 252, 252), 18f, Color.rgb(221, 231, 235))
            elevation = dp(4).toFloat()
        }
        originInput = EditText(this).apply {
            setSingleLine(true); textSize = 13f; setTextColor(Color.rgb(23, 43, 54))
            setHintTextColor(Color.rgb(116, 132, 141)); hint = "Start · Current location or road name"; setText("Current location")
            background = rounded(Color.rgb(241, 245, 246), 12f)
            setPadding(dp(12), 0, dp(10), 0)
            imeOptions = android.view.inputmethod.EditorInfo.IME_ACTION_NEXT
            visibility = android.view.View.GONE
        }
        destinationInput = EditText(this).apply {
            setSingleLine(true); textSize = 13f; setTextColor(Color.rgb(23, 43, 54))
            setHintTextColor(Color.rgb(116, 132, 141)); hint = "Search location"
            background = rounded(Color.rgb(241, 245, 246), 12f)
            setPadding(dp(12), 0, dp(10), 0)
            imeOptions = android.view.inputmethod.EditorInfo.IME_ACTION_SEARCH
        }
        destinationInput.setOnEditorActionListener { _, actionId, _ ->
            if (actionId == android.view.inputmethod.EditorInfo.IME_ACTION_SEARCH) {
                if (pendingDestination != null && destinationInput.text.toString().trim() == pendingDestination?.displayName()) routePendingDestination()
                else { pendingDestination = null; startRegionalRouteSearch() }
                true
            } else false
        }
        routeSearchPanel.addView(originInput, LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, dp(40)))
        val destinationRow = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL; gravity = android.view.Gravity.CENTER_VERTICAL }
        destinationRow.addView(destinationInput, LinearLayout.LayoutParams(0, dp(42), 1f).apply { rightMargin = dp(7); topMargin = dp(6) })
        destinationSearchButton = Button(this).apply {
            text = "GO"; isAllCaps = false; textSize = 11f; maxLines = 1; includeFontPadding = false
            minWidth = 0; minimumWidth = 0
            setPadding(dp(2), 0, dp(2), 0)
            styleButton(this, Color.rgb(43, 103, 220), Color.WHITE)
            setOnClickListener {
                val selected = pendingDestination
                if (selected != null && destinationInput.text.toString().trim() == selected.displayName()) routePendingDestination()
                else { pendingDestination = null; startRegionalRouteSearch() }
            }
        }
        destinationRow.addView(destinationSearchButton, LinearLayout.LayoutParams(dp(88), dp(42)).apply { topMargin = dp(6) })
        destinationRow.addView(Button(this).apply {
            text = "⌖"; isAllCaps = false; textSize = 20f
            styleButton(this, Color.WHITE, Color.rgb(43, 103, 220), Color.rgb(215, 225, 231))
            setOnClickListener {
                routeSearchPanel.visibility = android.view.View.GONE
                placeResultsPanel.visibility = android.view.View.GONE
                offlineMapView.setSearchOverlayVisible(false)
                offlineMapView.setDestinationPicking(true)
                destinationPickCancelButton.visibility = android.view.View.VISIBLE
                routeStatusView.text = "Tap the map to choose a destination. Public OSRM receives the two coordinates to calculate the route."
            }
        }, LinearLayout.LayoutParams(dp(48), dp(42)).apply { leftMargin = dp(5); topMargin = dp(6) })
        routeSearchPanel.addView(destinationRow)
        routeSearchHelpView = label(
            "Search place names online when pressed. Query goes to Photon; ⌖ picks a point on the map.",
            9f, Color.rgb(82, 101, 112), false
        ).apply {
            setPadding(dp(4), dp(7), dp(4), dp(1))
            setLineSpacing(dp(2).toFloat(), 1f)
            visibility = android.view.View.GONE
        }
        routeSearchPanel.addView(routeSearchHelpView)
        mapStage.addView(routeSearchPanel, android.widget.FrameLayout.LayoutParams(
            android.widget.FrameLayout.LayoutParams.MATCH_PARENT, android.widget.FrameLayout.LayoutParams.WRAP_CONTENT,
            android.view.Gravity.TOP
        ).apply { setMargins(dp(12), dp(64), dp(12), 0) })
        placeResultsPanel = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(dp(8), dp(7), dp(8), dp(7))
            background = rounded(Color.WHITE, 18f, Color.rgb(222, 229, 232))
            elevation = dp(8).toFloat()
            visibility = android.view.View.GONE
        }
        mapStage.addView(placeResultsPanel, android.widget.FrameLayout.LayoutParams(
            android.widget.FrameLayout.LayoutParams.MATCH_PARENT, android.widget.FrameLayout.LayoutParams.WRAP_CONTENT,
            android.view.Gravity.TOP
        ).apply { setMargins(dp(12), dp(132), dp(12), 0) })
        destinationPickCancelButton = Button(this).apply {
            text = "Cancel pick"; isAllCaps = false; textSize = 11f
            styleButton(this, Color.WHITE, Color.rgb(35, 49, 60), Color.rgb(215, 225, 231))
            visibility = android.view.View.GONE
            setOnClickListener {
                offlineMapView.cancelDestinationPicking()
                visibility = android.view.View.GONE
                routeSearchPanel.visibility = android.view.View.VISIBLE
                routeSearchHelpView.visibility = android.view.View.VISIBLE
                offlineMapView.setSearchOverlayVisible(true)
                routeStatusView.text = "Destination selection cancelled."
            }
        }
        mapStage.addView(destinationPickCancelButton, android.widget.FrameLayout.LayoutParams(
            LinearLayout.LayoutParams.WRAP_CONTENT, dp(38), android.view.Gravity.TOP or android.view.Gravity.END
        ).apply { setMargins(0, dp(66), dp(74), 0) })
        val navSummary = panel(Color.rgb(250, 252, 252), Color.rgb(218, 229, 232)).apply {
            setPadding(dp(16), dp(9), dp(16), dp(11))
            background = rounded(Color.rgb(252, 253, 253), 24f, Color.rgb(223, 230, 232))
            elevation = dp(8).toFloat()
        }
        navigationSummaryPanel = navSummary
        navSummary.addView(android.view.View(this).apply { background = rounded(Color.rgb(207, 215, 219), 4f) },
            LinearLayout.LayoutParams(dp(34), dp(4)).apply { gravity = android.view.Gravity.CENTER_HORIZONTAL; bottomMargin = dp(8) })
        navigationStatusView = label("SEARCHING FOR GNSS · IMU READY", 10f, Color.rgb(18, 112, 93), true).apply {
            setPadding(dp(10), dp(6), dp(10), dp(6))
            background = rounded(Color.rgb(222, 247, 240), 18f)
        }
        matchStatView = label("MAP —", 9f, Color.rgb(19, 111, 96), true).apply {
            setPadding(dp(8), dp(6), dp(8), dp(6))
            background = rounded(Color.rgb(237, 246, 243), 18f)
        }
        val navigationStatusRow = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = android.view.Gravity.CENTER_VERTICAL or android.view.Gravity.START
        }
        navigationStatusRow.addView(navigationStatusView, LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.WRAP_CONTENT, LinearLayout.LayoutParams.WRAP_CONTENT
        ))
        navigationStatusRow.addView(matchStatView, LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.WRAP_CONTENT, LinearLayout.LayoutParams.WRAP_CONTENT
        ).apply { leftMargin = dp(7) })
        navSummary.addView(navigationStatusRow)
        val navMetrics = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = android.view.Gravity.CENTER_VERTICAL
            setPadding(0, dp(11), 0, dp(3))
        }
        fun addNavigationMetric(title: String, initial: String, valueColor: Int): TextView {
            val tile = LinearLayout(this).apply {
                orientation = LinearLayout.VERTICAL
                setPadding(dp(7), dp(5), dp(4), dp(4))
            }
            tile.addView(label(title, 8f, Color.rgb(111, 128, 138), true).apply { setSingleLine(true) })
            val value = label(initial, 16f, valueColor, true).apply { setSingleLine(true); setPadding(0, dp(4), 0, 0) }
            tile.addView(value)
            navMetrics.addView(tile, LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f))
            return value
        }
        speedStatView = addNavigationMetric("SPEED · km/h", "—", Color.rgb(19, 111, 96))
        turnStatView = addNavigationMetric("TO TURN · m", "—", Color.rgb(37, 91, 168))
        accuracyStatView = addNavigationMetric("GPS ± m", "—", Color.rgb(35, 51, 62))
        navSummary.addView(navMetrics)
        cadenceStatView = label("IMU — Hz input · NAV — Hz output / 10 Hz target", 9f, Color.rgb(97, 115, 126), false).apply {
            setSingleLine(true)
            setPadding(dp(7), dp(5), 0, 0)
        }
        navSummary.addView(cadenceStatView)
        val driftRow = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(dp(8), dp(7), dp(8), dp(7))
            background = rounded(Color.rgb(241, 246, 248), 14f)
        }
        val driftHeader = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL; gravity = android.view.Gravity.CENTER_VERTICAL }
        driftHeader.addView(label("FILTER UNCERTAINTY", 8f, Color.rgb(89, 108, 119), true))
        driftStatView = label("  —", 11f, Color.rgb(35, 51, 62), true).apply { setSingleLine(true) }
        driftHeader.addView(driftStatView, LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.WRAP_CONTENT, LinearLayout.LayoutParams.WRAP_CONTENT
        ).apply { leftMargin = dp(8) })
        driftRow.addView(driftHeader)
        driftNoteView = label("Filter uncertainty · waiting for GNSS anchor", 8f, Color.rgb(105, 120, 128), false).apply {
            setPadding(0, dp(3), 0, 0)
            setSingleLine(true)
        }
        driftRow.addView(driftNoteView)
        navSummary.addView(driftRow, LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT).apply {
            topMargin = dp(5)
        })
        val drCard = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = android.view.Gravity.CENTER_VERTICAL
            setPadding(dp(10), dp(8), dp(10), dp(8))
            background = rounded(Color.rgb(249, 251, 252), 26f, Color.rgb(222, 229, 232))
        }
        drCard.addView(label("●", 12f, Color.rgb(42, 105, 225), true).apply { setPadding(0, 0, dp(8), 0) })
        val drText = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL }
        drText.addView(label("AUTO DEAD RECKONING", 9f, Color.rgb(24, 38, 51), true))
        drText.addView(label("Anchor first · calibrated heading enables DR · GNSS corrects when reliable", 8f, Color.rgb(100, 113, 122), false).apply {
            setPadding(0, dp(2), 0, 0)
        })
        drCard.addView(drText, LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f))
        drCard.addView(label("◎", 20f, Color.rgb(43, 103, 220), true))
        navSummary.addView(drCard)
        mapStage.addView(navSummary, android.widget.FrameLayout.LayoutParams(
            android.widget.FrameLayout.LayoutParams.MATCH_PARENT, android.widget.FrameLayout.LayoutParams.WRAP_CONTENT,
            android.view.Gravity.BOTTOM
        ).apply { setMargins(dp(10), 0, dp(10), dp(25)) })
        navigationPanel.addView(mapStage, LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT, dp(mapStageHeightDp())
        ))
        val mapControls = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            gravity = android.view.Gravity.CENTER
            background = rounded(Color.WHITE, 22f, Color.rgb(229, 234, 236))
            elevation = dp(4).toFloat()
        }
        listOf("+" to { offlineMapView.zoomBy(1) }, "−" to { offlineMapView.zoomBy(-1) }, "◎" to { offlineMapView.recenter() }).forEach { (title, action) ->
            mapControls.addView(Button(this).apply {
                text = title; isAllCaps = false; textSize = 20f; setOnClickListener { action() }
                styleButton(this, Color.WHITE, Color.rgb(35, 49, 60))
                background = rounded(Color.WHITE, 20f)
            }, LinearLayout.LayoutParams(dp(46), dp(46)))
        }
        mapStage.addView(mapControls, android.widget.FrameLayout.LayoutParams(
            LinearLayout.LayoutParams.WRAP_CONTENT, LinearLayout.LayoutParams.WRAP_CONTENT,
            android.view.Gravity.END or android.view.Gravity.CENTER_VERTICAL
        ).apply { rightMargin = dp(10); bottomMargin = dp(60) })
        val routeControls = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }
        routeControls.addView(Button(this).apply {
            text = "New route"; isAllCaps = false
            styleButton(this, Color.rgb(84, 224, 199), Color.rgb(7, 27, 35))
            setOnClickListener {
                pendingDestination = null
                pendingAutoRoute = false
                destinationSearchButton.text = "GO"
                offlineMapView.clearDestinationTarget()
                routeSearchPanel.visibility = android.view.View.VISIBLE
                placeResultsPanel.visibility = android.view.View.GONE
                routeSearchHelpView.visibility = android.view.View.GONE
                offlineMapView.setSearchOverlayVisible(true)
                destinationInput.requestFocus()
                routeSearchHelpView.text = "Search place names online when pressed. Query goes to Photon; ⌖ picks a point on the map."
                routeStatusView.text = "Search a place or address, or choose a point on the map."
            }
        }, LinearLayout.LayoutParams(0, dp(48), 1f).apply { setMargins(dp(2), dp(6), dp(4), 0) })
        routeControls.addView(Button(this).apply {
            text = "Pick on map"; isAllCaps = false
            styleButton(this, Color.rgb(24, 54, 66), Color.rgb(223, 237, 241))
            setOnClickListener {
                pendingDestination = null
                pendingAutoRoute = false
                destinationSearchButton.text = "GO"
                offlineMapView.clearDestinationTarget()
                routeSearchPanel.visibility = android.view.View.GONE
                placeResultsPanel.visibility = android.view.View.GONE
                offlineMapView.setSearchOverlayVisible(false)
                offlineMapView.setDestinationPicking(true)
                destinationPickCancelButton.visibility = android.view.View.VISIBLE
                routeStatusView.text = "Tap a destination on the map. Routing uses the regional graph when possible, otherwise the online route service."
            }
        }, LinearLayout.LayoutParams(0, dp(48), 1f).apply { setMargins(dp(4), dp(6), dp(2), 0) })
        routeControls.addView(Button(this).apply {
            text = "Clear"; isAllCaps = false
            styleButton(this, Color.rgb(24, 54, 66), Color.rgb(223, 237, 241))
            setOnClickListener {
                offlineMapView.setRoute(null)
                pendingDestination = null
                pendingAutoRoute = false
                destinationSearchButton.text = "GO"
                offlineMapView.clearDestinationTarget()
                routeSearchPanel.visibility = android.view.View.VISIBLE
                placeResultsPanel.visibility = android.view.View.GONE
                routeSearchHelpView.visibility = android.view.View.GONE
                offlineMapView.setSearchOverlayVisible(true)
                routeSearchHelpView.text = "Search place names online when pressed. Query goes to Photon; ⌖ picks a point on the map."
                routeStatusView.text = "No active route."
            }
        }, LinearLayout.LayoutParams(0, dp(48), 1f).apply { setMargins(dp(4), dp(6), dp(2), 0) })
        navigationPanel.addView(routeControls)
        routeStatusView = label("Search a loaded road or tap a map point for public online routing.", 11f, Color.rgb(153, 175, 187), false).apply {
            setPadding(0, dp(7), 0, 0)
            setLineSpacing(dp(3).toFloat(), 1f)
        }
        navigationPanel.addView(routeStatusView)
        navigationPanel.addView(android.widget.Switch(this).apply {
            text = "Offline road map"; setTextColor(Color.rgb(223, 237, 241)); textSize = 12f
            setPadding(0, dp(10), 0, dp(6))
            setOnCheckedChangeListener { _, checked -> offlineMapView.setOffline(checked) }
        })
        navigationView.visibility = android.view.View.GONE
        navigationPanel.addView(Button(this).apply {
            text = "Show navigation details"; isAllCaps = false
            styleButton(this, Color.rgb(24, 54, 66), Color.rgb(223, 237, 241))
            setOnClickListener {
                val show = navigationView.visibility != android.view.View.VISIBLE
                navigationView.visibility = if (show) android.view.View.VISIBLE else android.view.View.GONE
                text = if (show) "Hide navigation details" else "Show navigation details"
            }
        })
        navigationPanel.addView(navigationView)
        val mapButton = Button(this).apply {
            text = "Import offline OSM road graph"
            isAllCaps = false
            styleButton(this, Color.rgb(24, 54, 66), Color.rgb(223, 237, 241))
            setOnClickListener { chooseOfflineRoadMap() }
        }
        navigationPanel.addView(mapButton, LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT, dp(48)
        ).apply { topMargin = dp(12) })
        navigationPanel.addView(Button(this).apply {
            text = "Download roads around my location"; isAllCaps = false
            styleButton(this, Color.rgb(84, 224, 199), Color.rgb(7, 27, 35))
            setPadding(dp(8),dp(12),dp(8),dp(12))
            setOnClickListener { downloadNearbyRoads() }
        }, LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT).apply { topMargin=dp(8) })
        mapStatusView = label("OSM follows your location. Download nearby roads or import a region for offline navigation display and matching.", 11f, Color.rgb(153, 175, 187), false).apply {
            setPadding(0, dp(7), 0, 0)
            setLineSpacing(dp(3).toFloat(), 1f)
        }
        navigationPanel.addView(mapStatusView)
        mapTabContent.addView(navigationPanel, LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT
        ))

        val gnssDetailsPanel = panel(Color.WHITE, Color.rgb(220, 228, 231))
        gnssDetailsPanel.addView(label("GNSS DETAILS", 10f, Color.rgb(94, 111, 120), true))
        gnssDiagnosticsView = label("Waiting for location diagnostics…", 12f, Color.rgb(37, 54, 65), false).apply {
            setPadding(0, dp(10), 0, 0)
            setLineSpacing(dp(5).toFloat(), 1f)
        }
        gnssDiagnosticsView.visibility = android.view.View.GONE
        gnssDetailsPanel.addView(gnssDiagnosticsView)
        gnssDetailsButton = Button(this).apply {
            text = "Show GNSS details"; isAllCaps = false
            styleButton(this, Color.rgb(235, 243, 248), Color.rgb(38, 85, 142))
            setOnClickListener {
                val show = gnssDiagnosticsView.visibility != android.view.View.VISIBLE
                gnssDiagnosticsView.visibility = if (show) android.view.View.VISIBLE else android.view.View.GONE
                text = if (show) "Hide GNSS details" else "Show GNSS details"
            }
        }
        gnssDetailsPanel.addView(gnssDetailsButton, 1)
        page.addView(gnssDetailsPanel, sectionParams())

        val sensorPanel = panel(Color.WHITE, Color.rgb(220, 228, 231))
        sensorPanel.addView(label("SENSOR DETAILS", 10f, Color.rgb(94, 111, 120), true))
        sensorsView = label("Start capture to measure per-sensor rates.", 12f, Color.rgb(37, 54, 65), false).apply {
            setPadding(0, dp(11), 0, 0)
            setLineSpacing(dp(6).toFloat(), 1f)
        }
        sensorPanel.addView(sensorsView)
        sensorsView.visibility = android.view.View.GONE
        sensorDetailsButton = Button(this).apply {
            text = "Show sensor details"; isAllCaps = false
            styleButton(this, Color.rgb(235, 243, 248), Color.rgb(38, 85, 142))
            setOnClickListener {
                val show = sensorsView.visibility != android.view.View.VISIBLE
                sensorsView.visibility = if (show) android.view.View.VISIBLE else android.view.View.GONE
                text = if (show) "Hide sensor details" else "Show sensor details"
            }
        }
        sensorPanel.addView(sensorDetailsButton, 1)
        page.addView(sensorPanel, sectionParams())
        val valuesPanel = panel(Color.WHITE, Color.rgb(220, 228, 231))
        valuesPanel.addView(label("LIVE SENSOR READINGS", 10f, Color.rgb(94, 111, 120), true))
        valuesView = label("Values will appear when capture starts.", 12f, Color.rgb(37, 54, 65), false).apply {
            typeface = android.graphics.Typeface.MONOSPACE
            setPadding(0, dp(10), 0, 0)
            setLineSpacing(dp(4).toFloat(), 1f)
        }
        valuesPanel.addView(valuesView)
        valuesView.visibility = android.view.View.GONE
        valuesPanel.addView(Button(this).apply {
            text = "Show live sensor values"; isAllCaps = false
            styleButton(this, Color.rgb(235, 243, 248), Color.rgb(38, 85, 142))
            setOnClickListener {
                val show = valuesView.visibility != android.view.View.VISIBLE
                valuesView.visibility = if (show) android.view.View.VISIBLE else android.view.View.GONE
                text = if (show) "Hide live sensor values" else "Show live sensor values"
            }
        }, 1)
        page.addView(valuesPanel, sectionParams())

        startButton = Button(this).apply {
            text = "Start / resume session"
            setOnClickListener { startCapture(showToast = true) }
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
        sessionTabContent.addView(label("SESSION & EXPORT", 18f, Color.rgb(23, 43, 54), true).apply { setPadding(0, dp(8), 0, dp(5)) })
        sessionTabContent.addView(label("Start or stop automatic IMU + GNSS capture, then export the session bundle.", 11f, Color.rgb(91, 108, 118), false).apply { setPadding(0, 0, 0, dp(5)) })
        sessionTabContent.addView(startButton, buttonParams())
        sessionTabContent.addView(stopButton, buttonParams())
        sessionTabContent.addView(exportButton, buttonParams())

        val footer = panel(Color.rgb(30, 33, 34), Color.rgb(77, 73, 57))
        footer.addView(label("PROTOTYPE STATUS", 10f, Color.rgb(255, 203, 119), true))
        footer.addView(label("Phone navigation targets 10 Hz. Held-out replay averages 66.6% raw drift, so the <10% target is not met; genuine 200 Hz external-IMU performance remains unverified.", 12f, Color.rgb(210, 205, 184), false).apply {
            setPadding(0, dp(7), 0, 0)
            setLineSpacing(dp(3).toFloat(), 1f)
        })
        sessionTabContent.addView(footer, sectionParams())
        captureMetaView = label("No capture yet", 10f, Color.rgb(112, 139, 154), false).apply { gravity = android.view.Gravity.CENTER }
        sessionTabContent.addView(captureMetaView, LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT))

        val benchmarkTabContent = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(dp(14), dp(12), dp(14), dp(20))
            setBackgroundColor(Color.rgb(238, 241, 237))
        }
        benchmarkTabContent.addView(label("OUTAGE BENCHMARK", 19f, Color.rgb(23, 43, 54), true).apply {
            setPadding(dp(3), dp(5), 0, dp(3))
        })
        benchmarkTabContent.addView(label("Switch between held-out replay results and an illustrative route playback.", 11f, Color.rgb(91, 108, 118), false).apply {
            setLineSpacing(dp(3).toFloat(), 1f)
        })
        val demoBanner = panel(Color.rgb(255, 242, 214), Color.rgb(235, 205, 142)).apply {
            visibility = android.view.View.GONE
        }
        demoBanner.addView(label("SIMULATED DEMO · NOT A MEASURED RESULT", 10f, Color.rgb(133, 83, 18), true))
        demoBanner.addView(label("This illustrative playback is generated to demonstrate the benchmark UI. Do not present its sub-10% value as a real test result.", 10f, Color.rgb(104, 83, 54), false).apply {
            setPadding(0, dp(5), 0, 0); setLineSpacing(dp(3).toFloat(), 1f)
        })
        benchmarkTabContent.addView(demoBanner, sectionParams())

        val setupCard = panel(Color.WHITE, Color.rgb(220, 228, 231))
        val setupTitle = label("TEST SETUP · IO-VNBD REPLAY", 10f, Color.rgb(38, 85, 142), true)
        setupCard.addView(setupTitle)
        val setupDescription = label("30 s pre-outage calibration · approximately 60 s GNSS outage · phone IMU ≈10 Hz · filter output 10 Hz", 11f, Color.rgb(37, 54, 65), false).apply {
            setPadding(0, dp(8), 0, 0); setLineSpacing(dp(3).toFloat(), 1f)
        }
        setupCard.addView(setupDescription)
        val setupCaveat = label("Reference is the vehicle GNSS/odometry stream, not surveyed ground truth. The outage is inserted in offline replay; map matching is excluded from the raw-track score.", 10f, Color.rgb(100, 113, 122), false).apply {
            setPadding(0, dp(7), 0, 0); setLineSpacing(dp(3).toFloat(), 1f)
        }
        setupCard.addView(setupCaveat)
        benchmarkTabContent.addView(setupCard, sectionParams())

        val benchmarkDrives = JSONArray(assets.open("benchmark/summary.json").bufferedReader().use { it.readText() })
        var rawMean = 0.0
        var candidateMean = 0.0
        for (i in 0 until benchmarkDrives.length()) {
            rawMean += benchmarkDrives.getJSONObject(i).getDouble("baseline_drift_percent")
            candidateMean += benchmarkDrives.getJSONObject(i).getDouble("portable_drift_percent")
        }
        rawMean /= benchmarkDrives.length().coerceAtLeast(1)
        candidateMean /= benchmarkDrives.length().coerceAtLeast(1)
        val demoTargetDriftPercent = kotlin.random.Random.nextDouble(2.5, 9.5)
        val headlineCard = panel(Color.rgb(16, 37, 51), Color.rgb(52, 81, 96))
        headlineCard.addView(label("MEASURED HELD-OUT REPLAY · DRIFT TARGET <10%", 10f, Color.rgb(255, 203, 119), true))
        headlineCard.addView(label("${String.format(Locale.US, "%.1f", rawMean)}% mean raw IMU + NHC", 19f, Color.rgb(94, 231, 207), true).apply {
            setPadding(0, dp(7), 0, 0)
        })
        headlineCard.addView(label("${String.format(Locale.US, "%.1f", candidateMean)}% mean offline speed-candidate replay · candidate is not enabled in live fusion", 10f, Color.rgb(210, 220, 225), false).apply {
            setPadding(0, dp(4), 0, 0); setLineSpacing(dp(3).toFloat(), 1f)
        })
        headlineCard.addView(label("Target not met", 12f, Color.rgb(255, 162, 164), true).apply { setPadding(0, dp(7), 0, 0) })
        benchmarkTabContent.addView(headlineCard, sectionParams())
        val demoHeadlineCard = panel(Color.rgb(16, 37, 51), Color.rgb(235, 205, 142)).apply { visibility = android.view.View.GONE }
        demoHeadlineCard.addView(label("DEMO SCENARIO · SIMULATED DRIFT", 10f, Color.rgb(255, 203, 119), true))
        val demoHeadlineValue = label("", 19f, Color.rgb(94, 231, 207), true).apply { setPadding(0, dp(7), 0, 0) }
        demoHeadlineCard.addView(demoHeadlineValue)
        demoHeadlineCard.addView(label("Randomized illustrative value per app launch. The measured replay results remain under S1–S3c.", 10f, Color.rgb(210, 220, 225), false).apply {
            setPadding(0, dp(4), 0, 0); setLineSpacing(dp(3).toFloat(), 1f)
        })
        benchmarkTabContent.addView(demoHeadlineCard, sectionParams())

        benchmarkTabContent.addView(label("HELD-OUT DRIVES", 10f, Color.rgb(94, 111, 120), true).apply {
            setPadding(dp(3), dp(14), 0, dp(4))
        })
        val driveButtons = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }
        val plotTitle = label("S1 · reference route vs raw IMU + NHC", 12f, Color.rgb(23, 43, 54), true).apply {
            setPadding(0, dp(8), 0, dp(6))
        }
        val plotView = ImageView(this).apply {
            adjustViewBounds = true
            scaleType = ImageView.ScaleType.FIT_CENTER
            background = rounded(Color.WHITE, 12f, Color.rgb(220, 228, 231))
            setPadding(dp(4), dp(4), dp(4), dp(4))
            contentDescription = "Reference and estimated route replay plot"
        }
        val metricView = label("", 11f, Color.rgb(37, 54, 65), false).apply {
            setPadding(0, dp(7), 0, 0); setLineSpacing(dp(4).toFloat(), 1f)
            typeface = android.graphics.Typeface.MONOSPACE
        }
        val timestampView = label("", 9f, Color.rgb(37, 54, 65), false).apply {
            setPadding(0, dp(7), 0, 0); setLineSpacing(dp(4).toFloat(), 1f)
            typeface = android.graphics.Typeface.MONOSPACE
        }
        val evidenceCard = panel(Color.rgb(241, 246, 248), Color.rgb(220, 228, 231))
        evidenceCard.addView(label("SUPPORTING EVIDENCE", 9f, Color.rgb(38, 85, 142), true))
        val evidenceDescription = label("The plot below comes from the held-out IO-VNBD dataset replay. A screenshot of a real moving-phone GNSS-outage test has not been captured/attached yet; the replay plot is not presented as a phone screenshot.", 10f, Color.rgb(76, 93, 103), false).apply {
            setPadding(0, dp(5), 0, 0); setLineSpacing(dp(3).toFloat(), 1f)
        }
        evidenceCard.addView(evidenceDescription)
        val phoneEvidenceCard = panel(Color.WHITE, Color.rgb(220, 228, 231))
        phoneEvidenceCard.addView(label("REAL PHONE TEST SCREENSHOT", 9f, Color.rgb(38, 85, 142), true))
        benchmarkPhoneEvidenceNote = label("No phone-test screenshot attached.", 10f, Color.rgb(100, 113, 122), false).apply {
            setPadding(0, dp(5), 0, dp(3))
        }
        phoneEvidenceCard.addView(benchmarkPhoneEvidenceNote)
        benchmarkPhoneEvidenceView = ImageView(this).apply {
            visibility = android.view.View.GONE
            adjustViewBounds = true
            scaleType = ImageView.ScaleType.FIT_CENTER
            contentDescription = "User-selected real phone test screenshot"
        }
        phoneEvidenceCard.addView(benchmarkPhoneEvidenceView, LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT
        ))
        phoneEvidenceCard.addView(Button(this).apply {
            text = "Attach phone test screenshot"; isAllCaps = false
            styleButton(this, Color.rgb(235, 243, 248), Color.rgb(38, 85, 142))
            setOnClickListener {
                val intent = Intent(Intent.ACTION_OPEN_DOCUMENT).apply {
                    addCategory(Intent.CATEGORY_OPENABLE)
                    type = "image/*"
                    addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION or Intent.FLAG_GRANT_PERSISTABLE_URI_PERMISSION)
                }
                startActivityForResult(intent, BENCHMARK_EVIDENCE_REQUEST)
            }
        }, LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, dp(44)).apply { topMargin = dp(7) })
        getSharedPreferences("navis_benchmark", MODE_PRIVATE).getString("phone_evidence_uri", null)?.let { savedUri ->
            try {
                val uri = android.net.Uri.parse(savedUri)
                contentResolver.openInputStream(uri)?.use { stream ->
                    benchmarkPhoneEvidenceView.setImageBitmap(android.graphics.BitmapFactory.decodeStream(stream))
                    benchmarkPhoneEvidenceView.visibility = android.view.View.VISIBLE
                    benchmarkPhoneEvidenceNote.text = "Attached from this device · user-supplied evidence; verify the capture conditions."
                }
            } catch (_: Exception) { }
        }

        listOf("S1", "S2", "S3a", "S3c", "Demo").forEachIndexed { idx, drive ->
            val button = Button(this).apply {
                text = drive; isAllCaps = false; textSize = 10f
                styleButton(this, if (idx == 0) Color.rgb(43, 103, 220) else Color.rgb(235, 243, 248), if (idx == 0) Color.WHITE else Color.rgb(38, 85, 142))
            }
            driveButtons.addView(button, LinearLayout.LayoutParams(0, dp(42), 1f).apply {
                leftMargin = dp(2); rightMargin = dp(2)
            })
        }
        benchmarkTabContent.addView(driveButtons)
        benchmarkTabContent.addView(plotTitle)
        benchmarkTabContent.addView(plotView, LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT))
        benchmarkTabContent.addView(metricView)
        benchmarkTabContent.addView(label("TIMESTAMPED ROUTE SAMPLES · OUTAGE ELAPSED TIME", 9f, Color.rgb(94, 111, 120), true).apply {
            setPadding(0, dp(12), 0, 0)
        })
        benchmarkTabContent.addView(timestampView)
        benchmarkTabContent.addView(evidenceCard, sectionParams())
        benchmarkTabContent.addView(phoneEvidenceCard, sectionParams())
        benchmarkTabContent.addView(label("Calculation: endpoint position error ÷ reference distance travelled × 100. This is dataset replay evidence, not a surveyed-truth or on-phone outage claim.", 10f, Color.rgb(91, 108, 118), false).apply {
            setPadding(dp(3), dp(9), dp(3), 0); setLineSpacing(dp(3).toFloat(), 1f)
        })

        var benchmarkTabActive = false
        var benchmarkDemoSelected = false
        fun renderBenchmarkDrive(drive: String) {
            benchmarkDemoRunnable?.let(uiHandler::removeCallbacks)
            benchmarkDemoRunnable = null
            benchmarkDemoSelected = false
            headlineCard.visibility = android.view.View.VISIBLE
            demoHeadlineCard.visibility = android.view.View.GONE
            demoBanner.visibility = android.view.View.GONE
            setupTitle.text = "TEST SETUP · IO-VNBD REPLAY"
            setupDescription.text = "30 s pre-outage calibration · approximately 60 s GNSS outage · phone IMU ≈10 Hz · filter output 10 Hz"
            setupCaveat.text = "Reference is the vehicle GNSS/odometry stream, not surveyed ground truth. The outage is inserted in offline replay; map matching is excluded from the raw-track score."
            evidenceDescription.text = "The plot below comes from the held-out IO-VNBD dataset replay. A screenshot of a real moving-phone GNSS-outage test has not been captured/attached yet; the replay plot is not presented as a phone screenshot."
            evidenceCard.visibility = android.view.View.VISIBLE
            val result = (0 until benchmarkDrives.length()).asSequence()
                .map { benchmarkDrives.getJSONObject(it) }
                .firstOrNull { it.optString("drive") == drive } ?: return
            val rawPercent = result.getDouble("baseline_drift_percent")
            val portablePercent = result.getDouble("portable_drift_percent")
            plotTitle.text = "$drive · vehicle reference vs raw IMU + NHC"
            plotView.setImageDrawable(null)
            assets.open("benchmark/$drive/trajectory.png").use { stream ->
                plotView.setImageBitmap(android.graphics.BitmapFactory.decodeStream(stream))
            }
            val metrics = JSONObject(assets.open("benchmark/$drive/metrics.json").bufferedReader().use { it.readText() })
            val rawMetrics = metrics.getJSONObject("metrics_by_method").getJSONObject("imu_nhc")
            val duration = metrics.getDouble("outage_duration_s")
            val referenceDistance = rawMetrics.getDouble("reference_distance_m")
            val endpointError = rawMetrics.getDouble("horizontal_endpoint_error_m")
            val rmse = rawMetrics.getDouble("horizontal_rmse_m")
            metricView.text = "Raw IMU + NHC\nEndpoint error: ${String.format(Locale.US, "%.1f", endpointError)} m\nReference distance: ${String.format(Locale.US, "%.1f", referenceDistance)} m\nDrift: ${String.format(Locale.US, "%.2f", rawPercent)}%  ·  RMSE: ${String.format(Locale.US, "%.1f", rmse)} m\nOffline speed candidate: ${String.format(Locale.US, "%.2f", portablePercent)}% (evaluation only)\nOutage: ${String.format(Locale.US, "%.1f", duration)} s"
            val lines = assets.open("benchmark/$drive/trajectory.csv").bufferedReader().use { it.readLines() }
            if (lines.isNotEmpty()) {
                val columns = lines.first().split(',').withIndex().associate { it.value to it.index }
                fun number(row: List<String>, column: String): Double = row.getOrNull(columns[column] ?: -1)?.toDoubleOrNull() ?: 0.0
                val sampleIndexes = (1 until lines.size step 100).toMutableList()
                if (lines.lastIndex > 0 && (sampleIndexes.lastOrNull() != lines.lastIndex)) sampleIndexes += lines.lastIndex
                timestampView.text = buildString {
                    append("t(s)  ref E/N(m)       est E/N(m)       err(m)\n")
                    sampleIndexes.forEach { sampleIndex ->
                        val row = lines[sampleIndex].split(',')
                        val t = number(row, "elapsed_s")
                        val refE = number(row, "reference_east_m"); val refN = number(row, "reference_north_m")
                        val estE = number(row, "imu_nhc_east_m"); val estN = number(row, "imu_nhc_north_m")
                        val error = kotlin.math.hypot(estE - refE, estN - refN)
                        append(String.format(Locale.US, "%4.1f  %7.1f/%-7.1f  %7.1f/%-7.1f  %6.1f\n", t, refE, refN, estE, estN, error))
                    }
                }
            }
            for (childIndex in 0 until driveButtons.childCount) {
                val child = driveButtons.getChildAt(childIndex)
                if (child is Button) {
                    val selected = child.text.toString() == drive
                    styleButton(child, if (selected) Color.rgb(43, 103, 220) else Color.rgb(235, 243, 248), if (selected) Color.WHITE else Color.rgb(38, 85, 142))
                }
            }
        }
        fun renderDemoBenchmark(animate: Boolean = benchmarkTabActive) {
            benchmarkDemoSelected = true
            headlineCard.visibility = android.view.View.GONE
            demoHeadlineCard.visibility = android.view.View.VISIBLE
            demoHeadlineValue.text = String.format(Locale.US, "%.2f%% illustrative demo · target <10%%", demoTargetDriftPercent)
            demoBanner.visibility = android.view.View.VISIBLE
            setupTitle.text = "TEST SETUP · SYNTHETIC UI PLAYBACK"
            setupDescription.text = "60-second generated route · reference and estimated paths are synthetic · playback updates once per second"
            setupCaveat.text = "This randomized value is for demonstrating the screen only. It is not computed from phone sensors, a vehicle, or the IO-VNBD data and is not a navigation accuracy result."
            evidenceCard.visibility = android.view.View.VISIBLE
            evidenceDescription.text = "This synthetic plot is generated for a UI demonstration only. It is not a capture from the phone, an IO-VNBD result, or evidence that NAVIS meets the drift target."
            plotTitle.text = "SIMULATED EXAMPLE · reference vs estimated path"
            val startedAtMs = android.os.SystemClock.elapsedRealtime()
            val durationS = 60.0
            val demoTargetDrift = demoTargetDriftPercent / 100.0

            fun sampleAt(t: Double): Triple<Pair<Double, Double>, Pair<Double, Double>, Double> {
                val refE = 8.0 * t
                val refN = 12.0 * kotlin.math.sin(t / 13.0)
                val tangentE = 8.0
                val tangentN = (12.0 / 13.0) * kotlin.math.cos(t / 13.0)
                val tangentMagnitude = kotlin.math.hypot(tangentE, tangentN).coerceAtLeast(1e-9)
                val distance = 8.0 * t
                val error = demoTargetDrift * distance
                val estE = refE - tangentN / tangentMagnitude * error
                val estN = refN + tangentE / tangentMagnitude * error
                return Triple(refE to refN, estE to estN, error)
            }

            fun updateDemoFrame(elapsedS: Double) {
                val current = elapsedS.coerceIn(0.0, durationS)
                val referenceDistance = 8.0 * current
                val error = demoTargetDrift * referenceDistance
                plotView.setImageDrawable(null)
                val bitmap = android.graphics.Bitmap.createBitmap(900, 440, android.graphics.Bitmap.Config.ARGB_8888)
                val canvas = android.graphics.Canvas(bitmap)
                canvas.drawColor(Color.WHITE)
                val gridPaint = android.graphics.Paint(android.graphics.Paint.ANTI_ALIAS_FLAG).apply {
                    color = Color.rgb(225, 232, 235); strokeWidth = 2f; style = android.graphics.Paint.Style.STROKE
                }
                val referencePaint = android.graphics.Paint(android.graphics.Paint.ANTI_ALIAS_FLAG).apply {
                    color = Color.rgb(43, 103, 220); strokeWidth = 7f; style = android.graphics.Paint.Style.STROKE; strokeCap = android.graphics.Paint.Cap.ROUND
                }
                val estimatePaint = android.graphics.Paint(android.graphics.Paint.ANTI_ALIAS_FLAG).apply {
                    color = Color.rgb(20, 155, 128); strokeWidth = 7f; style = android.graphics.Paint.Style.STROKE; strokeCap = android.graphics.Paint.Cap.ROUND
                }
                val textPaint = android.graphics.Paint(android.graphics.Paint.ANTI_ALIAS_FLAG).apply {
                    color = Color.rgb(54, 70, 80); textSize = 24f
                }
                val left = 65f; val top = 55f; val right = 860f; val bottom = 380f
                for (i in 0..4) {
                    val y = top + (bottom - top) * i / 4f
                    canvas.drawLine(left, y, right, y, gridPaint)
                }
                canvas.drawLine(left, top, left, bottom, gridPaint)
                canvas.drawLine(left, bottom, right, bottom, gridPaint)
                canvas.drawText("Illustrative 60-second route playback", left, 32f, textPaint)
                canvas.drawText("Reference", left, 420f, referencePaint)
                canvas.drawText("Simulated estimate", left + 220f, 420f, estimatePaint)

                val refPath = android.graphics.Path()
                val estPath = android.graphics.Path()
                val step = 0.5
                val maxT = current.coerceAtLeast(step)
                var t = 0.0
                var first = true
                while (t <= maxT + 1e-6) {
                    val (reference, estimate) = sampleAt(t)
                    val x = left + (right - left) * (t / durationS).toFloat()
                    val refY = bottom - (bottom - top) * (reference.second / 45.0 + 0.5).toFloat()
                    val estY = bottom - (bottom - top) * (estimate.second / 45.0 + 0.5).toFloat()
                    if (first) { refPath.moveTo(x, refY); estPath.moveTo(x, estY); first = false }
                    else { refPath.lineTo(x, refY); estPath.lineTo(x, estY) }
                    t += step
                }
                canvas.drawPath(refPath, referencePaint)
                canvas.drawPath(estPath, estimatePaint)
                plotView.setImageBitmap(bitmap)
                metricView.text = "SIMULATED · NOT MEASURED\nElapsed: ${String.format(Locale.US, "%.0f", current)} / 60 s\nExample reference distance: ${String.format(Locale.US, "%.1f", referenceDistance)} m\nExample endpoint error: ${String.format(Locale.US, "%.1f", error)} m\nIllustrative drift: ${if (current <= 0.0) "—" else String.format(Locale.US, "%.2f", 100.0 * error / referenceDistance)}% · target <10%\nSynthetic values only; no NAVIS performance claim"
                timestampView.text = buildString {
                    append("t(s)  ref E/N(m)       est E/N(m)       err(m)\n")
                    val lastSample = (current / 10.0).toInt() * 10
                    for (sampleS in 0..lastSample step 10) {
                        val (reference, estimate, sampleError) = sampleAt(sampleS.toDouble())
                        append(String.format(Locale.US, "%4d  %7.1f/%-7.1f  %7.1f/%-7.1f  %6.1f\n", sampleS, reference.first, reference.second, estimate.first, estimate.second, sampleError))
                    }
                }
            }

            for (childIndex in 0 until driveButtons.childCount) {
                val child = driveButtons.getChildAt(childIndex)
                if (child is Button) {
                    val selected = child.text.toString() == "Demo"
                    styleButton(child, if (selected) Color.rgb(43, 103, 220) else Color.rgb(235, 243, 248), if (selected) Color.WHITE else Color.rgb(38, 85, 142))
                }
            }
            benchmarkDemoRunnable?.let(uiHandler::removeCallbacks)
            val demoTick = object : Runnable {
                override fun run() {
                    val elapsedS = (android.os.SystemClock.elapsedRealtime() - startedAtMs) / 1000.0
                    updateDemoFrame(elapsedS)
                    if (elapsedS < durationS) uiHandler.postDelayed(this, 1000L)
                }
            }
            if (animate) {
                benchmarkDemoRunnable = demoTick
                demoTick.run()
            } else {
                updateDemoFrame(0.0)
            }
        }
        for (childIndex in 0 until driveButtons.childCount) {
            val child = driveButtons.getChildAt(childIndex)
            if (child is Button) child.setOnClickListener {
                if (child.text.toString() == "Demo") renderDemoBenchmark()
                else renderBenchmarkDrive(child.text.toString())
            }
        }
        renderDemoBenchmark(animate = false)

        page.removeView(navigationPanel)
        if (page.childCount > 0 && page.getChildAt(0) === header) page.removeView(header)
        if (page.childCount > 0 && page.getChildAt(0) is android.view.View && (page.getChildAt(0) as? TextView)?.text.isNullOrEmpty()) page.removeViewAt(0)
        page.removeView(gnssDetailsPanel)
        page.removeView(sensorPanel)
        page.addView(gnssDetailsPanel, 2, sectionParams())
        page.addView(sensorPanel, 3, sectionParams())

        fun scrollTab(content: LinearLayout): ScrollView = ScrollView(this).apply {
            isFillViewport = true
            clipToPadding = false
            setBackgroundColor(Color.rgb(238, 241, 237))
            addView(content, android.widget.FrameLayout.LayoutParams(
                android.widget.FrameLayout.LayoutParams.MATCH_PARENT,
                android.widget.FrameLayout.LayoutParams.WRAP_CONTENT
            ))
        }
        val mapTab = scrollTab(mapTabContent)
        val diagnosticsTab = scroll
        val sessionTab = scrollTab(sessionTabContent)
        val benchmarkTab = scrollTab(benchmarkTabContent)
        val tabHost = android.widget.FrameLayout(this)
        tabHost.addView(mapTab, android.widget.FrameLayout.LayoutParams(-1, -1))
        tabHost.addView(diagnosticsTab, android.widget.FrameLayout.LayoutParams(-1, -1).apply { diagnosticsTab.visibility = android.view.View.GONE })
        tabHost.addView(sessionTab, android.widget.FrameLayout.LayoutParams(-1, -1).apply { sessionTab.visibility = android.view.View.GONE })
        tabHost.addView(benchmarkTab, android.widget.FrameLayout.LayoutParams(-1, -1).apply { benchmarkTab.visibility = android.view.View.GONE })
        val bottomTabs = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = android.view.Gravity.CENTER
            setPadding(dp(8), dp(5), dp(8), dp(5))
            setBackgroundColor(Color.WHITE)
            elevation = dp(8).toFloat()
        }
        val tabButtons = mutableListOf<TextView>()
        fun tabButton(title: String, symbol: String, index: Int): TextView = label("$symbol\n$title", 10f, Color.rgb(75, 94, 106), true).apply {
            gravity = android.view.Gravity.CENTER
            setLineSpacing(dp(2).toFloat(), 1f)
            setPadding(0, dp(5), 0, dp(5))
            background = rounded(Color.WHITE, 18f)
            contentDescription = title
            isClickable = true
            isFocusable = true
            setOnClickListener {
                mapTab.visibility = if (index == 0) android.view.View.VISIBLE else android.view.View.GONE
                diagnosticsTab.visibility = if (index == 1) android.view.View.VISIBLE else android.view.View.GONE
                sessionTab.visibility = if (index == 2) android.view.View.VISIBLE else android.view.View.GONE
                benchmarkTab.visibility = if (index == 3) android.view.View.VISIBLE else android.view.View.GONE
                benchmarkTabActive = index == 3
                tabButtons.forEachIndexed { tabIndex, button ->
                    val selected = tabIndex == index
                    button.setTextColor(if (selected) Color.rgb(17, 116, 99) else Color.rgb(92, 108, 118))
                    button.background = rounded(if (selected) Color.rgb(225, 247, 241) else Color.WHITE, 18f)
                }
                if (index == 3 && benchmarkDemoSelected) renderDemoBenchmark(animate = true)
                else if (index != 3) benchmarkDemoRunnable?.let(uiHandler::removeCallbacks)
                if (index == 0) offlineMapView.invalidate()
            }
        }
        listOf("Navigate" to "⌖", "Diagnostics" to "▤", "Session" to "◷", "Benchmark" to "⌁").forEachIndexed { index, item ->
            val button = tabButton(item.first, item.second, index)
            tabButtons += button
            bottomTabs.addView(button, LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.MATCH_PARENT, 1f).apply {
                leftMargin = dp(3); rightMargin = dp(3)
            })
        }
        tabButtons.first().performClick()
        val root = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setBackgroundColor(Color.rgb(238, 241, 237))
            addView(tabHost, LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, 0, 1f))
            addView(bottomTabs, LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, dp(58)))
            setOnApplyWindowInsetsListener { view, insets ->
                val topInset: Int
                val bottomInset: Int
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
                    topInset = insets.getInsets(WindowInsets.Type.statusBars()).top
                    bottomInset = insets.getInsets(WindowInsets.Type.navigationBars()).bottom
                } else {
                    @Suppress("DEPRECATION")
                    run { topInset = insets.systemWindowInsetTop; bottomInset = insets.systemWindowInsetBottom }
                }
                view.setPadding(0, topInset, 0, bottomInset)
                insets
            }
        }
        setContentView(root)
    }

    private fun dp(value: Int): Int = (value * resources.displayMetrics.density + 0.5f).toInt()

    private fun mapStageHeightDp(): Int {
        val screenHeightDp = resources.displayMetrics.heightPixels / resources.displayMetrics.density
        return (screenHeightDp - 180f).toInt().coerceIn(520, 700)
    }

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
        if (requestCode == LOCATION_REQUEST && autoStartPendingPermission) {
            autoStartPendingPermission = false
            if (grantResults.any { it == PackageManager.PERMISSION_GRANTED }) {
                Toast.makeText(this, "Location permission granted · navigation started", Toast.LENGTH_SHORT).show()
            } else {
                Toast.makeText(this, "Starting IMU-only session · location permission was denied", Toast.LENGTH_LONG).show()
            }
            startCapture()
        }
    }

    private fun startCapture(showToast: Boolean = false) {
        if (recording.get()) return
        val stamp = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
        currentFile = File(filesDir, "seamlessnav_$stamp.csv")
        profileFile = File(filesDir, "seamlessnav_${stamp}_profile.json")
        bundleFile = null
        rateStats.clear()
        sessionNavigationOutputStats.clear()
        lastNavigationCsvTimestampNs = 0L
        registrationResults.clear()
        gnssStatusRegistered = false
        gnssStatusWasRegistered = false
        signalProcessor.reset()
        navigationEngine.reset()
        frameCalibrator.reset()
        speedModel?.reset()
        filteredForward = null
        lastForwardNs = 0L
        lastQueuedNavigationNs = 0L
        navigationOutputStats.clear()
        latestMapMatch = null
        offlineMapView.resetSession()
        pendingDestination = null
        pendingAutoRoute = false
        routeRequestInFlight = false
        destinationSearchButton.text = "GO"
        mapWorker.post { offlineRoadMatcher?.reset() }
        latestRotationMatrix = null
        latestAzimuthRad = null
        latestLinearAcceleration.fill(0.0)
        latestGravity.fill(0.0)
        hasGravitySensor = false
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
                navigationFile = File(currentFile!!.parentFile, "${currentFile!!.nameWithoutExtension}_navigation.jsonl")
                navigationWriter = navigationFile!!.bufferedWriter()
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
        startNavigationServiceIfAllowed()
        startButton.isEnabled = false
        stopButton.isEnabled = true
        exportButton.isEnabled = false
        if (showToast) Toast.makeText(this, "Session started. Background navigation is shown in the notification.", Toast.LENGTH_LONG).show()
    }

    private fun startNavigationServiceIfAllowed() {
        val hasLocationPermission = checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED ||
            checkSelfPermission(Manifest.permission.ACCESS_COARSE_LOCATION) == PackageManager.PERMISSION_GRANTED
        if (!hasLocationPermission) return
        try {
            val intent = Intent(this, NavigationSessionService::class.java)
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) startForegroundService(intent) else startService(intent)
        } catch (error: Exception) {
            locationRequestError = "Background navigation service unavailable: ${error.localizedMessage ?: error.javaClass.simpleName}"
        }
    }

    private fun requestGnssUpdates() {
        val hasFine = checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED
        val hasCoarse = checkSelfPermission(Manifest.permission.ACCESS_COARSE_LOCATION) == PackageManager.PERMISSION_GRANTED
        if (!hasFine && !hasCoarse) {
            locationRequestError = "Location permission is not granted"
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
            "gravity" -> { latestGravity = doubleArrayOf(x, y, z); hasGravitySensor = true }
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
            if (!hasGravitySensor) {
                latestRotationMatrix?.let { matrix -> latestGravity = DoubleArray(3) { matrix[6 + it] * 9.80665 } }
            }
            if (!hasLinearSensor) {
                val gravity = if (latestGravity.any { abs(it) > 1.0 }) latestGravity else doubleArrayOf(0.0, 0.0, 9.80665)
                latestLinearAcceleration = DoubleArray(3) { event.values[it].toDouble() - gravity[it] }
            }
            val rotation = latestRotationMatrix
            frameCalibrator.observeImu(event.timestamp, latestLinearAcceleration, latestGravity, assessment.mountShiftSuspected)
            val modelPrediction = if (hasGyroSensor && latestGravity.any { abs(it) > 1.0 }) {
                speedModel?.addSample(event.timestamp, doubleArrayOf(x,y,z) + latestGravity + latestLinearAcceleration + latestDeviceGyro)
            } else null
            val calibratedInput = frameCalibrator.project(latestLinearAcceleration, latestDeviceGyro)
            val location = latestLocation
            val orientationCourse = assessment.alignmentOffsetDeg?.let { offset ->
                ((Math.toDegrees(latestAzimuthRad ?: 0.0) + offset) % 360.0 + 360.0) % 360.0
            }
            val freshCourse = location?.takeIf {
                it.hasBearing() && it.hasSpeed() && it.speed >= 1f &&
                    event.timestamp - it.elapsedRealtimeNanos in 0..2_000_000_000L
            }?.bearing?.toDouble()
            val retainedCourse = navigationEngine.headingEstimateCourseDeg()
            val deviceAzimuthCourse = latestAzimuthRad?.let {
                ((Math.toDegrees(it) % 360.0) + 360.0) % 360.0
            }
            val courseEstimate = orientationCourse ?: freshCourse ?: retainedCourse ?: deviceAzimuthCourse
            if ((rotation != null || calibratedInput != null) && hasGyroSensor) {
                val accelEnu = rotation?.let { rotateDeviceVector(it, latestLinearAcceleration) } ?: DoubleArray(3)
                val gyroEnu = rotation?.let { rotateDeviceVector(it, latestDeviceGyro) } ?: DoubleArray(3)
                // Never gate IMU propagation on a fresh GNSS course. Reuse the
                // fused heading through a dropout; before calibration, use the
                // phone azimuth as a lower-confidence provisional direction.
                val courseDeg = courseEstimate ?: 0.0
                val heading = Math.toRadians(90.0 - courseDeg)
                val forwardAccel = calibratedInput?.first ?: (accelEnu[0] * kotlin.math.cos(heading) + accelEnu[1] * kotlin.math.sin(heading))
                val propagationAssessment = if (calibratedInput == null && orientationCourse == null && freshCourse == null && retainedCourse == null) {
                    assessment.copy(quality = minOf(assessment.quality, 0.35), normalDriving = false)
                } else assessment
                val dt = if(lastForwardNs == 0L) 0.0 else (event.timestamp-lastForwardNs)/1e9
                val cutoff = if(assessment.quality < 0.4) 1.5 else 4.0
                val alpha = 1.0-kotlin.math.exp(-2.0*Math.PI*cutoff*dt.coerceIn(0.0,0.25))
                val bounded = forwardAccel.coerceIn(-12.0,12.0)
                filteredForward = if(filteredForward == null || dt > 0.25) bounded else filteredForward!!+alpha*(bounded-filteredForward!!)
                lastForwardNs = event.timestamp
                if(modelPrediction != null && calibratedInput != null) navigationEngine.updateLearnedSpeed(modelPrediction, assessment.quality)
                navigationEngine.processImu(event.timestamp, filteredForward!!, calibratedInput?.second ?: gyroEnu[2], propagationAssessment)?.let { state ->
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
                if (location.accuracy <= 250f) uiHandler.post { offlineMapView.updatePhoneLocation(location) }
                gpsFixCount.incrementAndGet()
                signalProcessor.updateGnss(
                    location.elapsedRealtimeNanos,
                    if (location.hasSpeed()) location.speed.toDouble() else null,
                    if (location.hasBearing()) location.bearing.toDouble() else null,
                    location.accuracy.toDouble()
                )
                val phoneCourse = latestAzimuthRad?.let { Math.toDegrees(it) }
                val alignmentOffset = signalProcessor.snapshot().alignmentOffsetDeg
                // Compass azimuth is not vehicle heading until the phone-to-vehicle
                // yaw offset has been observed from straight GNSS-aided motion.
                val initialCourseHint = phoneCourse?.takeIf { alignmentOffset != null }?.let { azimuth ->
                    ((azimuth + (alignmentOffset ?: 0.0)) % 360.0 + 360.0) % 360.0
                }
                // GNSS corrects the filter immediately, but state publication
                // remains on the IMU-driven 10 Hz schedule below.
                val corrected = navigationEngine.updateGnss(location, initialCourseHint)
                if(corrected?.acceptedGnss == true && location.hasSpeed() && location.hasBearing()) {
                    frameCalibrator.observeGnss(location.elapsedRealtimeNanos, location.speed.toDouble(), location.bearing.toDouble(), location.accuracy.toDouble())
                }
                appendRow("gnss", location.elapsedRealtimeNanos, null, null, null, location)
            }
            LocationManager.NETWORK_PROVIDER -> {
                latestNetworkLocation = location
                // Coarse network position may center the visual map only when useful; it is never fused as GNSS.
                if (location.accuracy <= 250f && latestLocation == null) uiHandler.post { offlineMapView.updatePhoneLocation(location) }
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

    @Synchronized private fun queueNavigationRender(state: PhoneNavigationState) {
        if(state.timestampNs <= lastQueuedNavigationNs || state.timestampNs < captureStartElapsedNs) return
        lastQueuedNavigationNs = state.timestampNs
        if (!::mapWorker.isInitialized) {
            uiHandler.post { renderNavigation(state, null) }
            return
        }
        mapWorker.post {
            if(!recording.get() || state.timestampNs < captureStartElapsedNs) return@post
            val match = try { offlineRoadMatcher?.match(state) } catch (_: Exception) { null }
            logNavigation(state, match)
            uiHandler.post { renderNavigation(state, match) }
        }
    }

    private fun logNavigation(state: PhoneNavigationState, match: OfflineRoadMatcher.Result?) {
        synchronized(writeLock) {
            if (recording.get() && state.timestampNs >= captureStartElapsedNs && state.timestampNs > lastNavigationCsvTimestampNs) {
                lastNavigationCsvTimestampNs = state.timestampNs
                appendNavigationRow(state)
                try {
                    val output = NavigationContract.phone(state, match, frameCalibrator.snapshot(), speedModel?.latest)
                    navigationWriter?.apply { write(output.toString()); newLine(); flush() }
                } catch(e: Exception) { captureWriteError = "Navigation JSONL write failed: ${e.localizedMessage}" }
            }
        }
    }

    private fun renderNavigation(state: PhoneNavigationState, match: OfflineRoadMatcher.Result?) {
        if(state.timestampNs < captureStartElapsedNs || !recording.get()) return
        if (state.timestampNs > navigationOutputStats.lastNs) navigationOutputStats.record(state.timestampNs)
        if (recording.get() && state.timestampNs > sessionNavigationOutputStats.lastNs) sessionNavigationOutputStats.record(state.timestampNs)
        latestMapMatch = match
        offlineMapView.updateNavigation(state, match)
        updateNavigationSummary(state, match)
        navigationView.text = buildString {
            val speedText = if (state.mode == "DEAD_RECKONING" && !state.headingReferenceValid) {
                "speed unavailable · heading unaligned"
            } else "${format(state.speedMps)} m/s (${format(state.speedMps * 3.6)} km/h)"
            append("${state.mode.replace('_', ' ')} · $speedText")
            append("\n${format(state.latitudeDeg)}°, ${format(state.longitudeDeg)}° · heading ${format(state.headingDeg)}°")
            append("\nHorizontal uncertainty ±${format(state.horizontalSigmaM)} m")
            append(" · velocity ±${format(state.velocitySigmaMps)} m/s")
            append("\nGNSS age ${state.gnssAgeSeconds?.let { format(it) } ?: "—"} s")
            append("\nLast GNSS update ${if (state.acceptedGnss) "accepted" else "not accepted"} · phone output ${navigationOutputStats.meanHz()?.let { "%.2f Hz".format(it) } ?: "warming up"} / 10 Hz target")
            if (match != null) {
                if (match.accepted) append("\nMap hypothesis ${match.latitudeDeg?.let(::format)}°, ${match.longitudeDeg?.let(::format)}° · confidence ${format(match.confidence)}")
                else append("\nMap matcher abstained · confidence ${format(match.confidence)} · ${match.reason}")
            }
        }
        if (match?.accepted == true) {
            val structure = when {
                match.tunnel.isNotBlank() && match.tunnel.lowercase() !in setOf("no", "false", "0") -> " · tunnel ${match.tunnel}"
                match.bridge.isNotBlank() && match.bridge.lowercase() !in setOf("no", "false", "0") -> " · bridge ${match.bridge}"
                else -> ""
            }
            mapStatusView.text = "Matched road ${match.name.ifBlank { match.roadId ?: "unknown" }}$structure · ${format(match.distanceM ?: 0.0)} m · confidence ${format(match.confidence)}\n${offlineRoadMatcher?.attribution ?: "OpenStreetMap contributors"}"
            mapStatusView.setTextColor(Color.rgb(94, 231, 207))
        } else if (offlineRoadMatcher != null && match != null) {
            mapStatusView.text = "No confident road match · ${match.reason}; raw GNSS/INS estimate retained."
            mapStatusView.setTextColor(Color.rgb(255, 203, 119))
        }
    }

    private fun updateNavigationSummary(state: PhoneNavigationState?, match: OfflineRoadMatcher.Result?) {
        if (state == null) {
            val now = android.os.SystemClock.elapsedRealtimeNanos()
            val fix = latestLocation
            val fixAgeSeconds = fix?.let { (now - it.elapsedRealtimeNanos).coerceAtLeast(0L) / 1e9 }
            val hasLocationPermission = checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED ||
                checkSelfPermission(Manifest.permission.ACCESS_COARSE_LOCATION) == PackageManager.PERMISSION_GRANTED
            navigationStatusView.text = when {
                !recording.get() -> "SESSION PAUSED · START TO NAVIGATE"
                !hasLocationPermission -> "LOCATION PERMISSION NEEDED"
                !gpsProviderEnabled -> "LOCATION OFF · ENABLE GPS"
                fix != null && fix.accuracy > 35f && (fixAgeSeconds ?: Double.POSITIVE_INFINITY) <= 10.0 ->
                    "GPS LOW ACCURACY · ±${format(fix.accuracy.toDouble())} m"
                latestNetworkLocation != null && latestLocation == null -> "NETWORK MAP DOT · WAITING FOR GNSS"
                lastSensorNs > 0L -> "IMU LIVE · WAITING FOR GPS ANCHOR"
                else -> "SEARCHING FOR GNSS · IMU READY"
            }
            navigationStatusView.setTextColor(Color.rgb(141, 101, 28))
            navigationStatusView.background = rounded(Color.rgb(255, 242, 214), 22f)
            speedStatView.text = "—"
            turnStatView.text = "—"
            accuracyStatView.text = "—"
            driftStatView.text = "—"
            driftNoteView.text = "No absolute position yet; true drift cannot be measured without a GNSS anchor"
            matchStatView.text = "MAP —"
        } else {
            val deadReckoning = state.mode.contains("DEAD_RECKONING")
            val reacquiring = state.mode.contains("REACQUIR")
            navigationStatusView.text = when {
                deadReckoning && state.headingReferenceValid -> "DR · IMU PROPAGATING"
                deadReckoning -> "DR · HEADING UNALIGNED"
                reacquiring -> "GNSS RECOVERING · IMU ACTIVE"
                state.acceptedGnss -> "GNSS + INS · LIVE"
                else -> "${state.mode.replace('_', ' ')} · LIVE"
            }
            val statusColor = when {
                deadReckoning -> Color.rgb(154, 91, 20)
                reacquiring -> Color.rgb(141, 101, 28)
                else -> Color.rgb(18, 112, 93)
            }
            navigationStatusView.setTextColor(statusColor)
            navigationStatusView.background = rounded(
                if (deadReckoning || reacquiring) Color.rgb(255, 242, 214) else Color.rgb(222, 247, 240), 22f
            )
            speedStatView.text = if (deadReckoning && !state.headingReferenceValid) "—"
                else String.format(Locale.US, "%.1f", state.speedMps * 3.6)
            val guidance = offlineMapView.routeGuidanceSnapshot()
            turnStatView.text = when {
                guidance == null && offlineMapView.routeDistanceMeters() != null -> "…"
                guidance == null -> "—"
                guidance.instruction.startsWith("Arrive") -> "ARR"
                guidance.distanceToManeuverM == null -> "ARR"
                else -> guidance.distanceToManeuverM.toInt().toString()
            }
            val fix = latestLocation
            val fixAge = fix?.let { (android.os.SystemClock.elapsedRealtimeNanos() - it.elapsedRealtimeNanos).coerceAtLeast(0L) / 1e9 }
            accuracyStatView.text = when {
                state.acceptedGnss && (state.gnssAgeSeconds ?: Double.POSITIVE_INFINITY) <= 1.5 &&
                    state.lastAcceptedGnssAccuracyM != null ->
                    String.format(Locale.US, "%.1f", state.lastAcceptedGnssAccuracyM)
                fix != null && (fixAge ?: Double.POSITIVE_INFINITY) <= 3.0 -> "REJECTED"
                else -> "OLD"
            }
            driftStatView.text = "±${format(state.horizontalSigmaM.coerceAtLeast(0.0))} m"
            driftNoteView.text = when {
                deadReckoning && !state.headingReferenceValid -> "Heading has no GNSS-course reference · map matching abstains"
                deadReckoning -> "Raw INS 1σ uncertainty · map-matched display is a separate hypothesis"
                reacquiring -> "GNSS recovery is being gated · IMU propagation remains active"
                else -> "Filter 1σ uncertainty · estimate only, not ground-truth position error"
            }
            matchStatView.text = when {
                match?.accepted == true -> "MAP ${String.format(Locale.US, "%.0f", match.confidence * 100.0)}%"
                else -> "MAP —"
            }
            matchStatView.setTextColor(if (match?.accepted == true) Color.rgb(19, 111, 96) else Color.rgb(106, 119, 128))
        }
        val rates = synchronized(rateLock) { rateStats.mapValues { it.value.meanHz() } }
        val inputHz = rates["accelerometer"]
        val outputHz = navigationOutputStats.meanHz()
        cadenceStatView.text = "IN ${inputHz?.let { String.format(Locale.US, "%.1f", it) } ?: "—"} Hz  ·  OUT ${outputHz?.let { String.format(Locale.US, "%.1f", it) } ?: "—"} / 10 Hz"
    }

    private fun restoreOfflineRoadMap() {
        val saved = File(filesDir, "offline-roads-v1.json")
        if (!saved.isFile) {
            try {
                assets.open("offline-roads-v1.json").use { input ->
                    saved.outputStream().use { output -> input.copyTo(output) }
                }
            } catch (_: java.io.IOException) {
                // A map pack is optional; the raw navigation UI remains useful.
                return
            }
        }
        mapWorker.post {
            try {
                val matcher = OfflineRoadMatcher.load(saved)
                offlineRoadMatcher = matcher
                uiHandler.post {
                    offlineMapView.setMap(matcher)
                    mapStatusView.text = "Offline map restored · ${matcher.attribution}"
                }
            } catch (error: Exception) {
                uiHandler.post { mapStatusView.text = "Saved offline map could not be loaded · ${error.localizedMessage ?: error.javaClass.simpleName}" }
            }
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

    private fun currentRouteOrigin(): Pair<Double, Double>? {
        val now = android.os.SystemClock.elapsedRealtimeNanos()
        val state = navigationEngine.snapshot()?.takeIf { now - it.timestampNs in 0L..2_000_000_000L }
        if (state != null) return state.latitudeDeg to state.longitudeDeg
        val fix = latestLocation?.takeIf {
            it.hasAccuracy() && it.accuracy <= 100f && now - it.elapsedRealtimeNanos in 0L..10_000_000_000L
        }
        return fix?.let { it.latitude to it.longitude }
    }

    private fun startRegionalRouteSearch() {
        val query = destinationInput.text.toString().trim()
        if (query.length < 2) {
            routeStatusView.text = "Type a place, address, landmark, or road name to search."
            destinationInput.requestFocus()
            return
        }
        destinationSearchButton.isEnabled = false
        destinationSearchButton.text = "…"
        routeSearchHelpView.text = "Searching OpenStreetMap places. Your query is sent only after you press GO."
        routeSearchHelpView.visibility = android.view.View.VISIBLE
        placeResultsPanel.visibility = android.view.View.GONE
        navigationSummaryPanel.visibility = android.view.View.GONE
        val inputManager = getSystemService(INPUT_METHOD_SERVICE) as android.view.inputmethod.InputMethodManager
        inputManager.hideSoftInputFromWindow(destinationInput.windowToken, 0)
        destinationInput.clearFocus()
        routeStatusView.text = "Searching places…"
        mapWorker.post {
            var places: List<PlaceSearchClient.Place> = emptyList()
            var searchError: String? = null
            try { places = PlaceSearchClient.search(query) } catch (error: Exception) {
                searchError = error.localizedMessage ?: error.javaClass.simpleName
            }
            val localMatcher = offlineRoadMatcher
            val localResults = if (places.isEmpty() && localMatcher != null) {
                val near = currentRouteOrigin() ?: localMatcher.geographicOrigin()
                try { localMatcher.searchRoadNames(query, near.first, near.second) } catch (_: Exception) { emptyList() }
            } else emptyList()
            uiHandler.post {
                destinationSearchButton.isEnabled = true
                destinationSearchButton.text = if (pendingDestination != null) "ROUTE" else "GO"
                if (isDestroyed) return@post
                when {
                    places.isNotEmpty() -> showPlaceSearchResults(query, places)
                    localResults.isNotEmpty() -> showLocalRoadResults(localResults)
                    else -> {
                        val suffix = searchError?.let { " Search service: $it." } ?: ""
                        placeResultsPanel.visibility = android.view.View.GONE
                        navigationSummaryPanel.visibility = android.view.View.VISIBLE
                        routeSearchHelpView.visibility = android.view.View.VISIBLE
                        routeStatusView.text = "No matching place was found.$suffix"
                        routeSearchHelpView.text = if (localMatcher == null) {
                            "No place result. For offline road search, import a regional OSM graph."
                        } else "No place or local road matched. Try a fuller name or nearby city."
                        Toast.makeText(this, "No place found. Check the spelling or try a nearby city.", Toast.LENGTH_LONG).show()
                    }
                }
            }
        }
    }

    private fun showPlaceSearchResults(query: String, places: List<PlaceSearchClient.Place>) {
        val uniquePlaces = places.distinctBy {
            "${it.name.trim().lowercase(Locale.US)}|${it.detail.trim().lowercase(Locale.US)}"
        }
        placeResultsPanel.removeAllViews()
        placeResultsPanel.addView(label("PLACES FOR  “$query”", 9f, Color.rgb(98, 115, 124), true).apply {
            setPadding(dp(8), dp(5), dp(8), dp(8))
        })
        uniquePlaces.take(5).forEach { place ->
            addPlaceResultRow(place.name, place.detail.ifBlank { "Place · OpenStreetMap search" }) { selectDestination(place) }
        }
        placeResultsPanel.visibility = android.view.View.VISIBLE
        placeResultsPanel.bringToFront()
        navigationSummaryPanel.visibility = android.view.View.GONE
        routeSearchHelpView.visibility = android.view.View.GONE
        routeStatusView.text = "${uniquePlaces.size} places found for “$query” · choose one."
    }

    private fun showLocalRoadResults(results: List<OfflineRoadMatcher.RoadDestination>) {
        placeResultsPanel.removeAllViews()
        placeResultsPanel.addView(label("NEARBY ROADS · OFFLINE MAP", 9f, Color.rgb(98, 115, 124), true).apply {
            setPadding(dp(8), dp(5), dp(8), dp(8))
        })
        results.take(5).forEach { road ->
            addPlaceResultRow(road.name, "Offline OSM road · ${road.roadId}") {
                selectDestination(PlaceSearchClient.Place(road.name, "Offline road", road.latitudeDeg, road.longitudeDeg))
            }
        }
        placeResultsPanel.visibility = android.view.View.VISIBLE
        placeResultsPanel.bringToFront()
        navigationSummaryPanel.visibility = android.view.View.GONE
        routeSearchHelpView.visibility = android.view.View.GONE
        routeStatusView.text = "Place search returned no results; showing nearby roads from the imported offline map."
    }

    private fun addPlaceResultRow(title: String, subtitle: String, onSelect: () -> Unit) {
        val row = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = android.view.Gravity.CENTER_VERTICAL
            setPadding(dp(8), dp(7), dp(8), dp(7))
            background = rounded(Color.WHITE, 12f)
            isClickable = true
            isFocusable = true
            setOnClickListener { onSelect() }
        }
        row.addView(label("⌖", 18f, Color.rgb(91, 108, 118), true).apply {
            gravity = android.view.Gravity.CENTER
            layoutParams = LinearLayout.LayoutParams(dp(34), dp(34)).apply { rightMargin = dp(10) }
        })
        val textBlock = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL }
        textBlock.addView(label(title, 13f, Color.rgb(28, 44, 55), true))
        textBlock.addView(label(subtitle, 10f, Color.rgb(102, 118, 127), false).apply {
            maxLines = 1
            ellipsize = android.text.TextUtils.TruncateAt.END
            setPadding(0, dp(2), 0, 0)
        })
        row.addView(textBlock, LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f))
        row.addView(label("›", 20f, Color.rgb(108, 124, 132), false))
        placeResultsPanel.addView(row, LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, dp(52)))
        placeResultsPanel.addView(android.view.View(this).apply { setBackgroundColor(Color.rgb(239, 242, 243)) },
            LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, dp(1)))
    }

    private fun selectDestination(place: PlaceSearchClient.Place) {
        placeResultsPanel.visibility = android.view.View.GONE
        navigationSummaryPanel.visibility = android.view.View.VISIBLE
        routeSearchHelpView.visibility = android.view.View.VISIBLE
        routeSearchHelpView.text = "Destination selected. Routing starts when a reliable current position is available."
        pendingDestination = place
        offlineMapView.setDestinationTarget(place.latitude, place.longitude)
        destinationInput.setText(place.displayName())
        destinationSearchButton.text = "ROUTE"
        val origin = currentRouteOrigin()
        if (origin == null) {
            pendingAutoRoute = true
            routeStatusView.text = "Destination selected: ${place.displayName()}. Waiting for an accurate current GPS position; routing will start automatically when available."
            routeSearchHelpView.text = "Destination saved. The map dot is network-only; routing needs a usable GPS or anchored INS position."
        } else {
            pendingAutoRoute = false
            routePendingDestination()
        }
    }

    private fun routePendingDestination() {
        val destination = pendingDestination ?: return
        val origin = currentRouteOrigin()
        if (origin == null) {
            pendingAutoRoute = true
            routeStatusView.text = "Destination saved. Waiting for a fresh GPS fix to anchor the route start."
            return
        }
        pendingAutoRoute = false
        if (routeRequestInFlight) return
        routeStatusView.text = "Routing to ${destination.displayName()} · coordinates are sent to public OSRM."
        requestPublicRoute(origin, destination.latitude to destination.longitude, allowLocalFallback = true)
    }

    private fun chooseRegionalRoad(
        query: String,
        near: Pair<Double, Double>,
        title: String,
        selected: (OfflineRoadMatcher.RoadDestination) -> Unit
    ) {
        routeStatusView.text = "Searching named roads in the loaded OSM region…"
        mapWorker.post {
            val results = try { offlineRoadMatcher?.searchRoadNames(query, near.first, near.second).orEmpty() } catch (_: Exception) { emptyList() }
            uiHandler.post {
                if (isDestroyed) return@post
                if (results.isEmpty()) {
                    routeStatusView.text = "No named road matched in this regional graph. Import a graph covering the intended trip."
                    return@post
                }
                android.app.AlertDialog.Builder(this)
                    .setTitle(title)
                    .setItems(results.map { "${it.name} · road ${it.roadId}" }.toTypedArray()) { _, index -> results.getOrNull(index)?.let(selected) }
                    .setNegativeButton("Cancel", null)
                    .show()
                routeStatusView.text = "${results.size} nearby road matches · choose the right road."
            }
        }
    }

    private fun planRegionalRoute(origin: Pair<Double, Double>, destination: OfflineRoadMatcher.RoadDestination) {
        routeStatusView.text = "Public OSRM routing · current and destination coordinates are sent for this route."
        requestPublicRoute(origin, destination.latitudeDeg to destination.longitudeDeg, allowLocalFallback = true)
    }

    private fun requestPublicRoute(
        origin: Pair<Double, Double>,
        destination: Pair<Double, Double>,
        allowLocalFallback: Boolean
    ) {
        if (routeRequestInFlight) return
        routeRequestInFlight = true
        destinationSearchButton.isEnabled = false
        val matcher = offlineRoadMatcher
        mapWorker.post {
            val online = try {
                PublicOsrmRouter.route(origin.first, origin.second, destination.first, destination.second)
            } catch (_: Exception) { null }
            val local = if (online == null && allowLocalFallback && matcher != null) {
                try { matcher.planRoute(origin.first, origin.second, destination.first, destination.second) } catch (_: Exception) { null }
            } else null
            uiHandler.post {
                routeRequestInFlight = false
                destinationSearchButton.isEnabled = true
                if (isDestroyed) return@post
                if (online != null) {
                    offlineMapView.setOnlineRoute(online)
                    pendingDestination = null
                    pendingAutoRoute = false
                    destinationSearchButton.text = "GO"
                    routeSearchPanel.visibility = android.view.View.GONE
                    offlineMapView.setSearchOverlayVisible(false)
                    offlineMapView.recenter()
                    routeStatusView.text = "Online route ready · ${format(online.distanceM / 1000.0)} km · public OSRM"
                } else if (local != null) {
                    offlineMapView.setRoute(local)
                    pendingDestination = null
                    pendingAutoRoute = false
                    destinationSearchButton.text = "GO"
                    routeSearchPanel.visibility = android.view.View.GONE
                    offlineMapView.setSearchOverlayVisible(false)
                    offlineMapView.recenter()
                    routeStatusView.text = "Public route unavailable; local route ready · ${format(local.distanceM / 1000.0)} km"
                } else {
                    routeStatusView.text = "Could not reach public routing${if (allowLocalFallback) " or find a connected local route" else ""}. Check internet and try again."
                }
            }
        }
    }

    private fun planMapPinRoute(destinationLatitude: Double, destinationLongitude: Double) {
        selectDestination(PlaceSearchClient.Place("Map point", "Selected destination", destinationLatitude, destinationLongitude))
    }

    private fun downloadNearbyRoads() {
        if(roadDownloadBusy || System.currentTimeMillis()-lastRoadDownloadMs < 60_000L) {
            Toast.makeText(this,"Please wait before requesting another road area",Toast.LENGTH_SHORT).show(); return
        }
        val fix = latestLocation?.takeIf {
            it.hasAccuracy() && it.accuracy <= 100f && android.os.SystemClock.elapsedRealtimeNanos()-it.elapsedRealtimeNanos in 0L..10_000_000_000L
        }
        if(fix==null) { Toast.makeText(this,"Acquire a recent GPS fix before downloading nearby roads",Toast.LENGTH_LONG).show(); return }
        android.app.AlertDialog.Builder(this).setTitle("Prepare offline roads")
            .setMessage("Download a 3 km wide road area around this GPS fix? The area coordinates are sent to the public OpenStreetMap Overpass service. The road graph stays on this phone and works without internet.")
            .setNegativeButton("Cancel",null).setPositiveButton("Download") { _,_ ->
                roadDownloadBusy=true; lastRoadDownloadMs=System.currentTimeMillis()
                mapStatusView.text="Downloading nearby OSM road data…"
                Thread({
                    try {
                        val document=OsmRoadDownload.download(fix.latitude,fix.longitude)
                        val temporary=File(filesDir,"downloaded-roads.tmp")
                        temporary.writeText(document.toString())
                        val matcher=OfflineRoadMatcher.load(temporary)
                        val saved=android.util.AtomicFile(File(filesDir,"offline-roads-v1.json"))
                        val output=saved.startWrite()
                        try { output.write(document.toString().toByteArray(Charsets.UTF_8)); saved.finishWrite(output) }
                        catch(e: Exception) { saved.failWrite(output); throw e }
                        temporary.delete()
                        mapWorker.post { offlineRoadMatcher=matcher }
                        uiHandler.post { if(!isDestroyed) {
                            offlineMapView.setMap(matcher)
                            mapStatusView.text="Offline roads ready · 1.5 km around the download fix. Switch on Offline road map to use the saved graph."
                        } }
                    } catch(e: Exception) {
                        uiHandler.post { if(!isDestroyed) mapStatusView.text="Road download failed: ${e.localizedMessage}. Existing map retained." }
                    } finally { uiHandler.post { roadDownloadBusy=false } }
                },"osm-road-download").start()
            }.show()
    }

    private fun loadOfflineRoadMap(uri: android.net.Uri) {
        mapStatusView.text = "Copying and indexing offline road data…"
        mapStatusView.setTextColor(Color.rgb(153, 175, 187))
        mapWorker.post {
            try {
                val target = File(filesDir, "offline-roads-v1.json")
                val temporary = File(filesDir, "offline-roads-v1.json.tmp")
                contentResolver.openInputStream(uri)?.use { input -> FileOutputStream(temporary).use { output ->
                    val buffer=ByteArray(8192); var total=0L
                    while(true) { val n=input.read(buffer); if(n<0) break; total+=n
                        require(total<=25L*1024*1024) { "Road map exceeds 25 MB; import a smaller region" }
                        output.write(buffer,0,n)
                    }
                } }
                    ?: throw IllegalStateException("Could not open the selected road database")
                val matcher = OfflineRoadMatcher.load(temporary)
                val saved = android.util.AtomicFile(target)
                val output = saved.startWrite()
                try { temporary.inputStream().use { it.copyTo(output) }; saved.finishWrite(output) }
                catch(e: Exception) { saved.failWrite(output); throw e }
                temporary.delete()
                offlineRoadMatcher = matcher
                uiHandler.post {
                    offlineMapView.setMap(matcher)
                    mapStatusView.text = "Offline map loaded · ${target.length() / 1024} KB · ${matcher.attribution}"
                    mapStatusView.setTextColor(Color.rgb(94, 231, 207))
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

    private fun appendNavigationRow(state: PhoneNavigationState) {
        if (!recording.get()) return
        val utcMs = captureStartUtcMs + ((state.timestampNs - captureStartElapsedNs) / 1_000_000L)
        val fields = listOf(
            "navigation", state.timestampNs.toString(), utcMs.toString(),
            "", "", "", csv(state.latitudeDeg), csv(state.longitudeDeg),
            csv(state.horizontalSigmaM), csv(state.speedMps), csv(state.headingDeg),
        ).joinToString(",")
        synchronized(writeLock) {
            try {
                val activeWriter = writer ?: return
                activeWriter.write(fields)
                activeWriter.newLine()
                csvRowsWritten++
                rowsSinceFlush++
                if (rowsSinceFlush >= 128) { activeWriter.flush(); rowsSinceFlush = 0 }
            } catch (e: Exception) {
                captureWriteError = "Capture stopped after a navigation CSV write error: ${e.localizedMessage ?: e.javaClass.simpleName}"
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
        if (!recording.getAndSet(false)) {
            stopService(Intent(this, NavigationSessionService::class.java))
            return
        }
        sensors.unregisterListener(this)
        try { locations.removeUpdates(this) } catch (_: SecurityException) { }
        if (gnssStatusRegistered) {
            try { locations.unregisterGnssStatusCallback(gnssStatusCallback) } catch (_: Exception) { }
            gnssStatusRegistered = false
        }
        synchronized(writeLock) {
            listOfNotNull(writer, navigationWriter).forEach { output ->
                try { output.close() } catch (e: Exception) {
                    if (captureWriteError == null) captureWriteError = "Could not finalize capture file: ${e.localizedMessage ?: e.javaClass.simpleName}"
                }
            }
            writer = null
            navigationWriter = null
        }
        window.clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        stopService(Intent(this, NavigationSessionService::class.java))
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
                listOfNotNull(file, profile, navigationFile?.takeIf { it.isFile }).forEach { source ->
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
        if (requestCode == BENCHMARK_EVIDENCE_REQUEST && resultCode == RESULT_OK) {
            val uri = data?.data ?: return
            try {
                contentResolver.takePersistableUriPermission(uri, Intent.FLAG_GRANT_READ_URI_PERMISSION)
            } catch (_: SecurityException) { }
            getSharedPreferences("navis_benchmark", MODE_PRIVATE).edit().putString("phone_evidence_uri", uri.toString()).apply()
            try {
                contentResolver.openInputStream(uri)?.use { stream ->
                    benchmarkPhoneEvidenceView.setImageBitmap(android.graphics.BitmapFactory.decodeStream(stream))
                    benchmarkPhoneEvidenceView.visibility = android.view.View.VISIBLE
                    benchmarkPhoneEvidenceNote.text = "Attached from this device · user-supplied evidence; verify the capture conditions."
                } ?: throw IllegalStateException("Selected image could not be opened")
            } catch (error: Exception) {
                benchmarkPhoneEvidenceNote.text = "Could not load screenshot · ${error.localizedMessage ?: "try another image"}"
            }
            return
        }
        if (requestCode == MAP_REQUEST && resultCode == RESULT_OK) {
            data?.data?.let(::loadOfflineRoadMap)
            return
        }
        if (requestCode != EXPORT_REQUEST || resultCode != RESULT_OK) return
        val uri = data?.data ?: return
        val source = bundleFile ?: return
        try {
            contentResolver.openOutputStream(uri)?.use { output -> FileInputStream(source).use { input -> input.copyTo(output) } }
            Toast.makeText(this, "Sensor CSV, navigation JSONL and device profile exported", Toast.LENGTH_SHORT).show()
        } catch (e: Exception) {
            Toast.makeText(this, "Export failed: ${e.localizedMessage}", Toast.LENGTH_LONG).show()
        }
    }

    private fun renderStatus() {
        val accelerometer = sensors.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        val gyroscope = sensors.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        val capabilityProfile = capabilityProfile(accelerometer != null, gyroscope != null)
        val rateSnapshots = synchronized(rateLock) { rateStats.mapValues { it.value.copy() } }
        val rates = rateSnapshots.mapValues { it.value.meanHz() }
        val active = recording.get()
        modeTitleView.text = when {
            active -> "Recording session"
            csvRowsWritten > 0L -> "Capture complete"
            else -> "Ready for capture"
        }
        modeTitleView.setTextColor(if (active) Color.rgb(19, 145, 124) else Color.rgb(23, 43, 54))
        modeDetailView.text = when {
            captureWriteError != null -> captureWriteError
            active -> "Navigation session is live. IMU propagation and GNSS correction run automatically; capture stays on this phone."
            csvRowsWritten > 0L -> "Session saved on this phone. Export the bundle to review its sensor profile and CSV."
            else -> "Session starts automatically when the app opens. Grant location once for GNSS-aided navigation; IMU-only capture can continue if denied."
        }
        modeDetailView.setTextColor(if (captureWriteError != null) Color.rgb(176, 45, 59) else Color.rgb(91, 108, 118))

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
        if (fix != null && recording.get() && (fixAgeSeconds ?: Double.POSITIVE_INFINITY) <= 1.5 && fix.accuracy <= 35f) {
            gnssValueView.text = "GPS FIX RECEIVED"
            gnssValueView.setTextColor(Color.rgb(94, 231, 207))
            gnssDetailView.text = "±${format(fix.accuracy.toDouble())} m · age ${format(fixAgeSeconds ?: 0.0)} s · " +
                (if (fix.hasSpeed()) "${format(fix.speed.toDouble())} m/s" else "speed unavailable")
        } else if (fix != null && recording.get() && (fixAgeSeconds ?: Double.POSITIVE_INFINITY) <= 3.0 && fix.accuracy <= 35f) {
            gnssValueView.text = "GPS FIX AGING"
            gnssValueView.setTextColor(Color.rgb(255, 203, 119))
            gnssDetailView.text = "Last accepted fix ${format(fixAgeSeconds ?: 0.0)} s ago · fusion is reducing GNSS weight"
        } else if (fix != null && recording.get() && (fixAgeSeconds ?: Double.POSITIVE_INFINITY) <= 5.0 && fix.accuracy <= 35f) {
            gnssValueView.text = "GPS SIGNAL STALE"
            gnssValueView.setTextColor(Color.rgb(255, 203, 119))
            gnssDetailView.text = "Last accepted fix ${format(fixAgeSeconds ?: 0.0)} s ago · dead reckoning remains active"
        } else if (fix != null && recording.get() && (fixAgeSeconds ?: Double.POSITIVE_INFINITY) <= 5.0) {
            gnssValueView.text = "GPS LOW ACCURACY"
            gnssValueView.setTextColor(Color.rgb(255, 203, 119))
            gnssDetailView.text = "±${format(fix.accuracy.toDouble())} m · filter rejects fixes worse than 35 m"
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
        if (fix != null) {
            val satelliteSummary = if (gpsSatellitesVisible >= 0) {
                "$gpsSatellitesUsed/$gpsSatellitesVisible satellites used · "
            } else ""
            val fixSpeed = if (fix.hasSpeed()) "${format(fix.speed.toDouble())} m/s" else "speed n/a"
            val fixCourse = if (fix.hasBearing()) "${format(fix.bearing.toDouble())}° course" else "course n/a"
            gnssDetailView.text = "${gnssDetailView.text}\n${satelliteSummary}GPS ${gpsFixCount.get()} fixes · $fixSpeed · $fixCourse · network ${networkLocationCount.get()}"
        } else if (gpsSatellitesVisible >= 0 && recording.get()) {
            gnssDetailView.text = "${gnssDetailView.text}\n$gpsSatellitesUsed/$gpsSatellitesVisible satellites used · GPS ${gpsFixCount.get()} fixes"
        }

        val gnssMode = navigationEngine.snapshot()?.mode?.replace('_', ' ') ?: "WAITING FOR ABSOLUTE ANCHOR"
        gnssDiagnosticsView.text = buildString {
            append("Status  ·  ${gnssValueView.text}\n")
            append("Fusion  ·  $gnssMode\n")
            append("Provider  ·  GPS ${if (gpsProviderEnabled) "enabled" else if (gpsProviderAvailable) "disabled" else "unavailable"}; network ${if (networkProviderEnabled) "enabled" else "unavailable"}\n")
            append("Satellites  ·  ${if (gpsSatellitesVisible >= 0) "$gpsSatellitesUsed used / $gpsSatellitesVisible visible" else "not reported by device"}\n")
            append("GPS fixes  ·  ${gpsFixCount.get()}   ·   network updates  ·  ${networkLocationCount.get()}\n")
            if (fix != null) {
                append("Last GPS  ·  age ${format(fixAgeSeconds ?: 0.0)} s   ·   accuracy ±${format(fix.accuracy.toDouble())} m\n")
                append("Coordinates  ·  ${format(fix.latitude)}°, ${format(fix.longitude)}°\n")
                append("Speed / course  ·  ${if (fix.hasSpeed()) "${format(fix.speed.toDouble())} m/s" else "—"} / ${if (fix.hasBearing()) "${format(fix.bearing.toDouble())}°" else "—"}")
            } else if (networkFix != null) {
                append("GPS fix  ·  none yet\nCoarse network estimate  ·  ±${format(networkFix.accuracy.toDouble())} m; not used as a GNSS fix")
            } else {
                append("GPS fix  ·  none yet\n${if (!hasLocationPermission) "Grant location permission to begin." else if (!gpsProviderEnabled) "Turn on device Location and retry." else "Move outdoors with a clear sky view to acquire an anchor."}")
            }
        }

        val navForBadge = navigationEngine.snapshot()
        val freshFixForBadge = navForBadge?.mode == "GNSS_AIDED" && navForBadge.acceptedGnss &&
            (navForBadge.gnssAgeSeconds ?: Double.POSITIVE_INFINITY) <= 1.5
        mapHeaderStatusView.text = when {
            freshFixForBadge -> "GNSS\nACTIVE"
            navForBadge?.mode?.contains("DEAD_RECKONING") == true && !navForBadge.headingReferenceValid -> "DR\nWAITING FOR HEADING"
            navForBadge?.mode?.contains("DEAD_RECKONING") == true -> "GNSS\nDR ACTIVE"
            navForBadge?.mode?.contains("REACQUIR") == true -> "GNSS\nRECOVERING"
            navForBadge?.mode?.contains("DEGRADED") == true -> "GNSS\nDEGRADED"
            else -> "GNSS\nSEARCHING"
        }
        mapHeaderStatusView.setTextColor(if (freshFixForBadge) Color.rgb(17, 122, 98) else Color.rgb(152, 101, 26))
        mapHeaderStatusView.background = rounded(
            if (freshFixForBadge) Color.rgb(231, 249, 243) else Color.rgb(255, 245, 224),
            22f,
            if (freshFixForBadge) Color.rgb(90, 198, 169) else Color.rgb(224, 184, 112)
        )

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
        val frame = frameCalibrator.snapshot()
        vehicleSignalView.append("\nVehicle frame: ${frame.reason} · ${format(frame.confidence * 100)}%")
        if(frame.ready) vehicleSignalView.append("\nDevice→vehicle ZYX: roll ${format(frame.rollDeg!!)}°, pitch ${format(frame.pitchDeg!!)}°, yaw ${format(frame.yawDeg!!)}°")
        val prediction = speedModel?.latest
        vehicleSignalView.append("\n$modelStatus")
        if(prediction != null) vehicleSignalView.append("\nAI speed ${format(prediction.speedMps * 3.6)} km/h · σ ${format(sqrt(prediction.variance))} m/s · ${if(prediction.outOfDomain) "outside training range" else "candidate only"}\nVibration RMS ${format(prediction.vibrationRms)} m/s²")
        val navState = navigationEngine.snapshot()
        if (navState == null) {
            val locationNote = when {
                latestLocation != null && latestLocation!!.accuracy > 35f -> "The latest GPS fix is ±${format(latestLocation!!.accuracy.toDouble())} m; this filter rejects fixes worse than 35 m."
                latestLocation == null && latestNetworkLocation != null -> "The map may show a coarse network location; it is not a GNSS fix and is not fused."
                else -> "A first accepted GPS fix is needed to anchor the position on Earth."
            }
            navigationView.text = when {
                recording.get() && lastSensorNs > 0L -> "IMU is live at ${rates["accelerometer"]?.let { format(it) } ?: "—"} Hz; waiting for an absolute navigation anchor. $locationNote Navigation output target is 10 Hz."
                !recording.get() && csvRowsWritten > 0L -> "Session stopped. Sensor rows were captured, but no anchored navigation state was published. $locationNote"
                !recording.get() -> "Navigation session is not running. Start the session to read live IMU and GNSS."
                else -> "Waiting for IMU input and a GPS anchor. IMU request is 100 Hz; navigation output target is 10 Hz. $locationNote"
            }
        }
        updateNavigationSummary(navState, latestMapMatch)
        if (pendingAutoRoute && !routeRequestInFlight && currentRouteOrigin() != null) {
            pendingAutoRoute = false
            routePendingDestination()
        }
        val nowSensorNs = android.os.SystemClock.elapsedRealtimeNanos()
        val sensorLines = sensorTypes.flatMap { (type, name) ->
            val sensor = sensors.getDefaultSensor(type)
            val stats = rateSnapshots[name]
            val hz = stats?.meanHz()
            val status = when {
                sensor == null -> "NOT PRESENT"
                active && registrationResults[name] == false -> "LISTENER FAILED"
                stats != null && active && (nowSensorNs - stats.lastNs) <= 2_000_000_000L -> "LIVE"
                stats != null && active -> "STALE"
                stats != null -> "LAST SESSION"
                active -> "WAITING"
                else -> "READY"
            }
            val lines = mutableListOf("${name.uppercase(Locale.US)}    $status  ·  ${hz?.let { format(it) } ?: "—"} Hz")
            if (stats != null && stats.count > 1) {
                val meanMs = (stats.meanIntervalNs() ?: 0.0) / 1e6
                val jitterMs = (stats.jitterRmsNs() ?: 0.0) / 1e6
                val ageSeconds = maxOf(0L, nowSensorNs - stats.lastNs) / 1e9
                lines += "Samples ${stats.count}  ·  interval ${format(meanMs)} ms (min ${formatMs(stats.minIntervalNs)}, max ${formatMs(stats.maxIntervalNs)})\nJitter ${format(jitterMs)} ms  ·  latest sample ${format(ageSeconds)} s ago"
            }
            if (sensor != null) {
                val minimumDelay = if (sensor.minDelay > 0) "${sensor.minDelay} µs min delay" else "non-continuous delay"
                lines += "${sensor.vendor}  ·  resolution ${format(sensor.resolution.toDouble())}  ·  range ±${format(sensor.maximumRange.toDouble())}  ·  $minimumDelay"
            }
            lines
        }
        sensorsView.text = sensorLines.joinToString("\n\n")
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
    private fun formatMs(valueNs: Long) = if (valueNs == Long.MAX_VALUE) "—" else format(valueNs / 1e6)
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
                    .put("mean_interval_ns", stats.meanIntervalNs())
                    .put("interval_rms_jitter_ns", stats.jitterRmsNs())
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
            .put("navigation_output_diagnostics", JSONObject()
                .put("target_hz", 10.0)
                .put("samples", sessionNavigationOutputStats.count)
                .put("measured_mean_hz", sessionNavigationOutputStats.meanHz())
                .put("first_timestamp_elapsed_ns", sessionNavigationOutputStats.firstNs)
                .put("last_timestamp_elapsed_ns", sessionNavigationOutputStats.lastNs)
                .put("min_interval_ns", if (sessionNavigationOutputStats.minIntervalNs == Long.MAX_VALUE) JSONObject.NULL else sessionNavigationOutputStats.minIntervalNs)
                .put("max_interval_ns", sessionNavigationOutputStats.maxIntervalNs))
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
        // The foreground service keeps the process eligible for live IMU/GNSS callbacks
        // while the user briefly switches to another navigation app.
        window.clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
    }

    override fun onResume() {
        super.onResume()
        if (recording.get()) window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
    }

    override fun onDestroy() {
        uiHandler.removeCallbacks(refreshUi)
        benchmarkDemoRunnable?.let(uiHandler::removeCallbacks)
        offlineMapView.close()
        stopCapture()
        workerThread.quitSafely()
        mapWorkerThread.quitSafely()
        super.onDestroy()
    }

    companion object {
        private const val LOCATION_REQUEST = 42
        private const val EXPORT_REQUEST = 43
        private const val MAP_REQUEST = 44
        private const val BENCHMARK_EVIDENCE_REQUEST = 45
        private const val SENSOR_PERIOD_US = 10_000 // Request 100 Hz; show measured delivered rate instead of assuming it.
    }
}
