package org.sih.seamlessnav

import org.json.JSONObject
import org.junit.Assert.*
import org.junit.Test
import kotlin.math.*

class NavigationModulesTest {
    private fun json(name: String) = JSONObject(javaClass.classLoader!!.getResourceAsStream(name)!!.bufferedReader().readText())

    @Test fun gnssDegradationDoesNotFlickerOnOneGoodFix() {
        val quality = GnssQualityHysteresis()
        quality.onAcceptedFix(5.0)
        assertFalse(quality.degraded)

        quality.markDegraded()
        quality.onAcceptedFix(5.0)
        assertTrue(quality.degraded)
        quality.onAcceptedFix(6.0)
        assertTrue(quality.degraded)
        quality.onAcceptedFix(7.0)
        assertFalse(quality.degraded)
    }

    @Test fun poorOrBorderlineFixRestartsGnssRecoveryStreak() {
        val quality = GnssQualityHysteresis()
        quality.markDegraded()
        quality.onAcceptedFix(6.0)
        quality.onAcceptedFix(16.0)
        quality.onAcceptedFix(6.0)
        quality.onAcceptedFix(6.0)
        assertTrue(quality.degraded)
        quality.onAcceptedFix(6.0)
        assertFalse(quality.degraded)
    }

    @Test fun portableInferenceMatchesPython() {
        val fixture=json("speed-parity.json")
        val a=fixture.getJSONArray("channels")
        val rows=(0 until a.length()).map { i->DoubleArray(4) { a.getJSONArray(i).getDouble(it) } }
        val result=PortableSpeedModel(json("speed-model-v1.json")).predictChannels(rows,2_000_000_000L)
        val expected=fixture.getJSONObject("prediction")
        assertEquals(expected.getDouble("speed_mps"),result.speedMps,1e-8)
        assertEquals(expected.getDouble("variance_m2ps2"),result.variance,1e-8)
        assertEquals(expected.getBoolean("out_of_domain"),result.outOfDomain)
        assertFalse(result.eligibleForFusion)
    }

    @Test fun modelUsesCompletedBinsAndResetsAfterGap() {
        val model=PortableSpeedModel(json("speed-model-v1.json"))
        val raw=doubleArrayOf(0.0,0.0,9.80665,0.0,0.0,9.80665,0.0,0.0,0.0,0.0,0.0,0.0)
        for(i in 0..199) assertNull(model.addSample(1_000_000_000L+i*10_000_000L,raw))
        assertNotNull(model.addSample(3_000_000_000L,raw))
        assertNull(model.addSample(4_000_000_000L,raw))
        assertNull(model.latest)
    }

    @Test fun mountAlignmentRecoversDifferentProperRotations() {
        for(yaw in listOf(0.0,0.8,-2.2)) for(pitch in listOf(0.0,0.7)) {
            val c=cos(yaw); val s=sin(yaw); val cp=cos(pitch); val sp=sin(pitch)
            val rotation=doubleArrayOf(c*cp,-s,c*sp,s*cp,c,s*sp,-sp,0.0,cp)
            val forward=rotation.sliceArray(0..2); val up=rotation.sliceArray(6..8)
            val calibrator=VehicleFrameCalibrator()
            for(i in 0..1100) {
                val t=1_000_000_000L+i*10_000_000L
                calibrator.observeImu(t,forward,DoubleArray(3) {up[it]*9.80665},false)
                if(i%100==0) calibrator.observeGnss(t,5.0+i/100.0,20.0,3.0)
            }
            val result=calibrator.snapshot()
            assertTrue(result.reason,result.ready)
            assertArrayEquals(rotation,result.rotation!!,1e-6)
            val projected=calibrator.project(DoubleArray(3) {2*forward[it]},DoubleArray(3) {0.2*up[it]})!!
            assertEquals(2.0,projected.first,1e-6); assertEquals(0.2,projected.second,1e-6)
            calibrator.observeImu(13_000_000_000L,forward,DoubleArray(3) {up[it]*9.80665},true)
            assertFalse(calibrator.snapshot().ready)
        }
    }

    @Test fun constantSpeedDoesNotInventMountAlignment() {
        val calibrator=VehicleFrameCalibrator()
        for(i in 0..1000) {
            val t=1_000_000_000L+i*10_000_000L
            calibrator.observeImu(t,DoubleArray(3),doubleArrayOf(0.0,0.0,9.80665),false)
            if(i%100==0) calibrator.observeGnss(t,10.0,0.0,3.0)
        }
        assertFalse(calibrator.snapshot().ready)
    }

    @Test fun osmConversionPreservesStructuresAndOneWayDirection() {
        val input=JSONObject("""{"elements":[{"type":"way","id":42,"nodes":[1,2],"geometry":[{"lat":51.5,"lon":-0.12},{"lat":51.501,"lon":-0.12}],"tags":{"highway":"primary","oneway":"-1","tunnel":"yes","layer":"-1","name":"Tunnel"}},{"type":"way","id":43,"nodes":[3,4],"geometry":[{"lat":51.5,"lon":-0.121},{"lat":51.501,"lon":-0.121}],"tags":{"highway":"primary","bridge":"yes","layer":"1"}}]}""")
        val arcs=OsmRoadDownload.convert(input,51.5,-0.12).getJSONArray("arcs")
        assertEquals(3,arcs.length())
        assertEquals("2",arcs.getJSONObject(0).getString("start_node"))
        assertEquals("yes",arcs.getJSONObject(0).getString("tunnel"))
        assertEquals(-1,arcs.getJSONObject(0).getInt("layer"))
        assertEquals("yes",arcs.getJSONObject(1).getString("bridge"))
    }

    @Test(expected=IllegalArgumentException::class) fun incompleteOverpassResponseIsRejected() {
        OsmRoadDownload.convert(JSONObject("""{"remark":"timeout","elements":[]}"""),51.5,-0.12)
    }

    @Test fun singleDistantRoadCannotCreateFalseConfidence() {
        val data=JSONObject("""{"elements":[{"type":"way","id":1,"nodes":[1,2],"geometry":[{"lat":51.499,"lon":-0.12},{"lat":51.501,"lon":-0.12}],"tags":{"highway":"primary","oneway":"yes"}}]}""")
        val temporary=java.io.File.createTempFile("roads-test-",".json")
        try {
            temporary.writeText(OsmRoadDownload.convert(data,51.5,-0.12).toString())
            val matcher=OfflineRoadMatcher.load(temporary)
            val offset=Math.toDegrees(24.0/(6378137*cos(Math.toRadians(51.5))))
            val distant=matcher.match(PhoneNavigationState(1_000_000_000,51.5,-0.12+offset,5.0,0.0,1.0,"GNSS_AIDED",true))
            assertFalse(distant.accepted)
            assertTrue(distant.reason.contains("absolute"))
            matcher.reset()
            assertTrue(matcher.match(PhoneNavigationState(2_000_000_000,51.5,-0.12,5.0,0.0,1.0,"GNSS_AIDED",true)).accepted)
        } finally { temporary.delete() }
    }

    @Test fun offlineRouteFollowsConnectedOneWayRoadsAndProvidesGuidance() {
        val input=JSONObject("""{"elements":[
            {"type":"way","id":51,"nodes":[1,2],"geometry":[{"lat":51.5,"lon":-0.12},{"lat":51.5,"lon":-0.119}],"tags":{"highway":"residential","name":"First Street"}},
            {"type":"way","id":52,"nodes":[2,3],"geometry":[{"lat":51.5,"lon":-0.119},{"lat":51.501,"lon":-0.119}],"tags":{"highway":"residential","name":"Second Street"}}
        ]}""")
        val temporary=java.io.File.createTempFile("route-test-",".json")
        try {
            temporary.writeText(OsmRoadDownload.convert(input,51.5,-0.12).toString())
            val matcher=OfflineRoadMatcher.load(temporary)
            val route=matcher.planRoute(51.5,-0.12,51.501,-0.119)
            assertNotNull("connected OSM streets should route",route)
            assertTrue(route!!.distanceM>150.0)
            assertTrue(route.maneuvers.isNotEmpty())
            assertTrue(route.maneuvers.first().instruction.startsWith("Turn"))
            val guidance=matcher.guidance(route,51.5,-0.119)
            assertTrue(guidance.deviationM<5.0)
            assertTrue(guidance.distanceRemainingM in 0.0..route.distanceM)
        } finally { temporary.delete() }
    }
}
