#pragma once
#include <cmath>
#include <limits>
#include <rclcpp/rclcpp.hpp>

#define EARTH_RADIUS 6371000.0 // Raio da Terra em metros

typedef struct{
    float lat;
    float lon;
    float alt;
    bool updated;
} GPSpos;

const GPSpos EMPTY_GPS = {
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::quiet_NaN(),
    false
};

bool isEmptyGPS(const GPSpos& gps) {
    return std::isnan(gps.lat) || std::isnan(gps.lon) || std::isnan(gps.alt) || !gps.updated;
}

// Converter graus para radianos
double toRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

// Calcular distância usando fórmula de Haversine
double calculateGPSDistance(const GPSpos& pos1, const GPSpos& pos2) {
    double lat1_rad = toRadians(pos1.lat);
    double lat2_rad = toRadians(pos2.lat);
    double delta_lat = toRadians(pos2.lat - pos1.lat);
    double delta_lon = toRadians(pos2.lon - pos1.lon);
    
    double a = sin(delta_lat/2) * sin(delta_lat/2) +
               cos(lat1_rad) * cos(lat2_rad) *
               sin(delta_lon/2) * sin(delta_lon/2);
    
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    
    // Distância horizontal em metros
    double horizontal_distance = EARTH_RADIUS * c;
    
    // Adicionar diferença de altitude (se disponível)
    double altitude_diff = abs(pos2.alt - pos1.alt);
    
    // Distância 3D usando Pitágoras
    double total_distance = sqrt(horizontal_distance * horizontal_distance + 
                                altitude_diff * altitude_diff);
    
    return total_distance;
}

bool am_i_far(GPSpos current, GPSpos target, float threshold_meters = 5.0f) {
    // Verificar se algum GPS está vazio
    if (isEmptyGPS(target)) {
        RCLCPP_DEBUG(rclcpp::get_logger("GPS_Handler"), 
                     "GPS position is empty - considering as far");
        return true;
    }
    
    // Calcular distância real usando Haversine
    double distance = calculateGPSDistance(current, target);
    
    bool is_far = distance > threshold_meters;
    
    RCLCPP_INFO(rclcpp::get_logger("GPS_Handler"), 
                "GPS Distance: %.2f meters (threshold: %.2f) - %s", 
                distance, threshold_meters, is_far ? "FAR" : "CLOSE");
    
    return is_far;
}