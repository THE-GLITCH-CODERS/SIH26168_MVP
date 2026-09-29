import java.util.Properties

plugins {
    id("com.android.application")
}

val localConfig = Properties().apply {
    val localPropertiesFile = rootProject.file("local.properties")
    if (localPropertiesFile.isFile) localPropertiesFile.inputStream().use { load(it) }
}
val cartoKey = localConfig.getProperty("carto.basemaps.key", "").trim()
val escapedCartoKey = cartoKey.replace("\\", "\\\\").replace("\"", "\\\"")

dependencies {
    testImplementation("junit:junit:4.13.2")
    testImplementation("org.json:json:20240303")
}

android {
    namespace = "org.sih.seamlessnav"
    compileSdk = 37

    buildFeatures {
        buildConfig = true
    }

    defaultConfig {
        applicationId = "org.sih.seamlessnav"
        minSdk = 26
        targetSdk = 35
        versionCode = 1
        versionName = "0.1.0"
        buildConfigField("String", "CARTO_BASEMAP_KEY", "\"$escapedCartoKey\"")
    }
}
