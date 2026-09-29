package org.sih.seamlessnav

import org.json.JSONArray
import org.json.JSONObject

/** Versioned raw-state contract shared with the JSONL edge engine. */
object NavigationContract {
    fun phone(state: PhoneNavigationState, match: OfflineRoadMatcher.Result?, frame: VehicleFrameCalibrator.Result,
              prediction: PortableSpeedModel.Prediction?): JSONObject = JSONObject()
        .put("schema", "seamlessnav-navigation-v1").put("source", "phone")
        .put("timestamp_ns", state.timestampNs).put("target_output_hz",10.0)
        .put("position",JSONObject().put("latitude_deg",state.latitudeDeg).put("longitude_deg",state.longitudeDeg)
            .put("east_m",state.eastM).put("north_m",state.northM))
        .put("velocity",JSONObject().put("east_mps",state.eastVelocityMps).put("north_mps",state.northVelocityMps)
            .put("speed_mps",state.speedMps).put("heading_deg_north_clockwise",state.headingDeg))
        .put("uncertainty",JSONObject().put("horizontal_sigma_m",state.horizontalSigmaM).put("velocity_sigma_mps",state.velocitySigmaMps))
        .put("mode",state.mode).put("gnss_age_s",state.gnssAgeSeconds ?: JSONObject.NULL).put("last_gnss_accepted",state.acceptedGnss)
        .put("last_accepted_gnss_accuracy_m",state.lastAcceptedGnssAccuracyM ?: JSONObject.NULL)
        .put("heading_reference_valid",state.headingReferenceValid)
        .put("map_hypothesis",match?.let { JSONObject().put("accepted",it.accepted).put("confidence",it.confidence)
            .put("latitude_deg",it.latitudeDeg ?: JSONObject.NULL).put("longitude_deg",it.longitudeDeg ?: JSONObject.NULL)
            .put("road_id",it.roadId ?: JSONObject.NULL).put("name",it.name).put("tunnel",it.tunnel).put("bridge",it.bridge).put("reason",it.reason) } ?: JSONObject.NULL)
        .put("calibration",JSONObject().put("ready",frame.ready).put("confidence",frame.confidence).put("observations",frame.observations)
            .put("sensor_to_vehicle_rotation",frame.rotation?.let { JSONArray(it.toList()) } ?: JSONObject.NULL)
            .put("assumption","gravity defines vehicle-up on near-level road").put("reason",frame.reason))
        .put("learned_speed",prediction?.let { JSONObject().put("timestamp_ns",it.timestampNs).put("speed_mps",it.speedMps)
            .put("variance_m2ps2",it.variance).put("eligible_for_fusion",it.eligibleForFusion).put("out_of_domain",it.outOfDomain)
            .put("vibration_rms_mps2",it.vibrationRms) } ?: JSONObject.NULL)
}
