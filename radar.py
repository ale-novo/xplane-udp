import math
import numpy
import pygame
import sys
from XPlaneUdp import *

def round_custom(value):
    #return round(value * 2) / 2
    return round(value * 5) / 5

def sweep_motion(simtime_value, sweep_time, wxr_angle):
    cycle_fraction = (simtime_value % sweep_time) / sweep_time
    
    if cycle_fraction <= 0.5:
        angle = -wxr_angle + (cycle_fraction / 0.5) * (2 * wxr_angle)
    else:
        angle = wxr_angle - ((cycle_fraction - 0.5) / 0.5) * (2 * wxr_angle)

    return round_custom(angle)

def draw_squares(screen, wxr_range, max_range, window_size, display_angle):
    min_side_length = window_size[1] / (max_range * 1) #2 # Smaller base size for squares at the start
    max_side_length = 1*window_size[1] / max_range  # Larger size for squares at the end

    radians_angle = math.radians(display_angle)  # Convert angle to radians for math calculations

    for key, color_value in wxr_range[display_angle].items():
        if color_value <= 50:
            color = (0, int((color_value / 50) * 255), 0)
        elif color_value <= 75:
            green_to_yellow_ratio = (color_value - 50) / 25
            color = (int(green_to_yellow_ratio * 255), 255, 0)
        else:
            yellow_to_red_ratio = (color_value - 75) / 25
            color = (255, int((1 - yellow_to_red_ratio) * 255), 0)

        # Calculate dynamic side length based on position
        proportion = key / max_range
        dynamic_side_length = min_side_length + (max_side_length - min_side_length) * proportion

        base_x_pos = window_size[0] / 2
        base_y_pos = window_size[1]

        # Adjust positions based on dynamic side length and angle
        x_pos = base_x_pos + proportion * window_size[1] * math.sin(radians_angle) - (dynamic_side_length / 2)
        y_pos = base_y_pos - proportion * window_size[1] * math.cos(radians_angle) - dynamic_side_length

        pygame.draw.rect(screen, color, (int(x_pos), int(y_pos), int(dynamic_side_length), int(dynamic_side_length)))


def haversine(lat1, lon1, lat2, lon2):
    R = 6371 * 0.539957

    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    delta_lat_rad = math.radians(lat2 - lat1)
    delta_lon_rad = math.radians(lon2 - lon1)
    
    a = math.sin(delta_lat_rad / 2) ** 2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon_rad / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    
    return distance

def wxr_ahead(wxr, lat, lon, heading, rng, error):
    start_lat_rad = numpy.radians(lat)
    start_lon_rad = numpy.radians(lon)

    end_lat_rad = numpy.radians(wxr['lat'])
    end_lon_rad = numpy.radians(wxr['lon'])

    delta_lon = end_lon_rad - start_lon_rad

    x = numpy.sin(delta_lon) * numpy.cos(end_lat_rad)
    y = numpy.cos(start_lat_rad) * numpy.sin(end_lat_rad) - numpy.sin(start_lat_rad) * numpy.cos(end_lat_rad) * numpy.cos(delta_lon)
    initial_bearing = numpy.arctan2(x, y)
    initial_bearing_deg = numpy.degrees(initial_bearing)

    bearing = (initial_bearing_deg + 360) % 360

    '''
    bearing_lower = (heading - error) % 360
    bearing_upper = (heading + error) % 360

    is_within_margin = False

    if bearing_lower <= bearing_upper:
        is_within_margin = bearing_lower <= bearing <= bearing_upper
    else:
        is_within_margin = bearing <= bearing_upper or bearing >= bearing_lower

    if is_within_margin:
    '''

    distance = haversine(lat, lon, wxr['lat'], wxr['lon'])

    if distance <= rng:
        return {
            'acf_lat': lat,
            'acf_lon': lon,
            'heading': heading,
            'wxr_lat': wxr['lat'],
            'wxr_lon': wxr['lon'],
            'bearing': round_custom(bearing),
            'lvl': wxr['storm_level'],
            'height': wxr['storm_height'],
            'range': round(distance),
        }

    #return None

if __name__ == '__main__':

  longitude = "sim/flightmodel/position/longitude"
  latitude = "sim/flightmodel/position/latitude"
  mag_track = "sim/cockpit2/gauges/indicators/ground_track_mag_pilot"
  mag_var = "sim/flightmodel/position/magnetic_variation"
  sim_time = "sim/time/local_time_sec"

  xp = XPlaneUdp()

  try:
    beacon = xp.FindIp()
    print(beacon)
    
    xp.AddDataRef(longitude)
    xp.AddDataRef(latitude)
    xp.AddDataRef(mag_track)
    xp.AddDataRef(mag_var)
    xp.AddDataRef(sim_time)

    xp.StartRadar(2000)
    max_range = 100
    wxr_angle = 180
    #wxr_angle = 50

    #wxr_range = {angle: {} for angle in range(-wxr_angle, wxr_angle + 1)}
    #wxr_range = {angle / 2: {} for angle in range(-2 * wxr_angle, 2 * wxr_angle + 1)}

    wxr_range = {angle / 5: {} for angle in range(-5 * wxr_angle, 5 * wxr_angle + 1)}
    #wxr_range = {angle / 5: {} for angle in range(0, int(359 * 5) + 1)}

    sweep_angle = 50
    sweep_time = 10

    pygame.init()
    window_size = (1024, 768)

    screen = pygame.display.set_mode(window_size)
    pygame.display.set_caption("Radar")

    while True:
      try:

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        values = xp.GetValues()

        latitude_value = values[latitude]
        longitude_value = values[longitude]
        mag_track_value = values[mag_track]
        mag_var_value = values[mag_var]
        wxr_value = values['RADR']
        simtime_value = values[sim_time]

        map_heading_value = round_custom(mag_track_value - mag_var_value)

        if wxr_value != []:
            for item in wxr_value:
                ahead = wxr_ahead(item, latitude_value, longitude_value, map_heading_value, max_range, wxr_angle)
                if ahead is not None:
                    #print('heading', map_heading_value, 'bearing', ahead['bearing'])
                    #rel_angle = map_heading_value - ahead['bearing']
                    #rel_angle = (raw_diff + 180) % 360 - 180

                    raw_diff = ahead['bearing'] - map_heading_value
                    rel_angle = round((raw_diff + 180) % 360 - 180, 1)

                    wxr_range[rel_angle][ahead['range']] = ahead['lvl']

        #display_angle = sweep_motion(simtime_value, sweep_time, wxr_angle)
        display_angle = sweep_motion(simtime_value, sweep_time, sweep_angle)

        #if display_angle == wxr_angle:
        #  screen.fill((0, 0, 0))  # Clear the screen with black
        
        draw_squares(screen, wxr_range, max_range, window_size, display_angle)
        
        pygame.display.flip()  

      except XPlaneTimeout:
        print("XPlane Timeout")
        exit(0)

    pygame.quit()
    sys.exit()

  except XPlaneVersionNotSupported:
    print("XPlane Version not supported.")
    exit(0)

  except XPlaneIpNotFound:
    print("XPlane IP not found. Probably there is no XPlane running in your local network.")
    exit(0)

