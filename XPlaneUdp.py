# Class to get dataref values from XPlane Flight Simulator via network. 
# License: GPLv3

import socket
import struct
import binascii
from time import sleep
import platform
import math

class XPlaneIpNotFound(Exception):
  args="Could not find any running XPlane instance in network."

class XPlaneTimeout(Exception):
  args="XPlane timeout."

class XPlaneVersionNotSupported(Exception):
  args="XPlane version not supported."

class XPlaneUdp:
  '''
  Get data from XPlane via network.
  Use a class to implement RAI Pattern for the UDP socket. 
  '''
  #constants
  MCAST_GRP = "239.255.1.1"
  MCAST_PORT = 49707 # (MCAST_PORT was 49000 for XPlane10)
  
  def __init__(self):
    # Open a UDP Socket to receive on Port 49000
    self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.socket.settimeout(3.0)
    # list of requested datarefs with index number
    self.datarefidx = 0
    self.datarefs = {} # key = idx, value = dataref
    # values from xplane
    self.BeaconData = {}
    self.xplaneValues = {}
    self.defaultFreq = 10

  def __del__(self):
    for i in range(len(self.datarefs)):
      self.AddDataRef(next(iter(self.datarefs.values())), freq=0)
    self.socket.close()

  def StartRadar(self, ppf):
    self.wxr_map_temp = {}
    self.wxr_map = {}

    self.wxr_lonmin = None
    self.wxr_lonmax = None
    self.wxr_latmin = None
    self.wxr_latmax = None

    self.wxr_ncol = 0
    self.wxr_nlin = 0

    cmd = b"RADR\x00"
    byte_ppf = bytes(str(ppf), 'utf-8')

    message = struct.pack("<4sx10s", cmd, byte_ppf)
    assert(len(message)==15)
    self.socket.sendto(message, (self.BeaconData["IP"], self.BeaconData["Port"]))

  def StopRadar(self):
    self.weather_map = {}
    self.StartRadar(0)

  def SendFpl(self, fpln):
    cmd = b"FPLN\x00"
    message = struct.pack("<5s1400s", cmd, fpln.encode())
    self.socket.sendto(message, (self.BeaconData["IP"], self.BeaconData["Port"]))

  def SendCommand(self, cmd):
    msg = 'CMND0'
    msg += cmd
    self.socket.sendto(msg.encode("latin_1"), (self.BeaconData["IP"], self.UDP_PORT))

  def WriteDataRef(self,dataref,value,vtype='float'):
    '''
    Write Dataref to XPlane
    DREF0+(4byte byte value)+dref_path+0+spaces to complete the whole message to 509 bytes
    DREF0+(4byte byte value of 1)+ sim/cockpit/switches/anti_ice_surf_heat_left+0+spaces to complete to 509 bytes
    '''
    cmd = b"DREF\x00"
    dataref  =dataref+'\x00'
    string = dataref.ljust(500).encode()
    message = "".encode()
    if vtype == "float":
      message = struct.pack("<5sf500s", cmd,value,string)
    elif vtype == "int":
      message = struct.pack("<5si500s", cmd, value, string)
    elif vtype == "bool":
      message = struct.pack("<5sI500s", cmd, int(value), string)

    assert(len(message)==509)
    self.socket.sendto(message, (self.BeaconData["IP"], self.UDP_PORT))

  def AddDataRef(self, dataref, freq = None):
    '''
    Configure XPlane to send the dataref with a certain frequency.
    You can disable a dataref by setting freq to 0. 
    '''
    idx = -9999

    if freq == None:
      freq = self.defaultFreq

    if dataref in self.datarefs.values():
      idx = list(self.datarefs.keys())[list(self.datarefs.values()).index(dataref)]
      if freq == 0:
        if dataref in self.xplaneValues.keys():
          del self.xplaneValues[dataref]
        del self.datarefs[idx]
    else:
      idx = self.datarefidx
      self.datarefs[self.datarefidx] = dataref
      self.datarefidx += 1
    
    cmd = b"RREF\x00"
    string = dataref.encode()
    message = struct.pack("<5sii400s", cmd, freq, idx, string)
    assert(len(message)==413)
    self.socket.sendto(message, (self.BeaconData["IP"], self.BeaconData["Port"]))
    if (self.datarefidx%100 == 0):
      sleep(0.2)

  def GetValues(self):
    try:
      # Receive packet
      data, addr = self.socket.recvfrom(1472) # maximum bytes of an answer X-Plane will send (Ethernet MTU 1500b - IP hdr 20b - UDP hdr 8b)
      # Decode Packet
      retvalues = {}
      # * Read the Header "RREFO".
      header=data[0:5]
      #print(str(header))
      if(header==b"RREF,"): # (was b"RREFO" for XPlane10)
        # * We get 8 bytes for every dataref sent:
        #   An integer for idx and the float value. 
        values = data[5:]
        lenvalue = 8
        numvalues = int(len(values)/lenvalue)
        for i in range(0,numvalues):
          singledata = data[(5+lenvalue*i):(5+lenvalue*(i+1))]
          (idx,value) = struct.unpack("<if", singledata)
          if idx in self.datarefs.keys():
            # convert -0.0 values to positive 0.0 
            if value < 0.0 and value > -0.001 :
              value = 0.0
            retvalues[self.datarefs[idx]] = value
            
      elif(header==b"RADR5"):
        '''
        XP11
        (header,       # == 'RADR'
         lon,          # float longitude of radar point
         lat,          # float latitude
         storm_level,  # precipitation level, 0 to 100
         storm_height  # storm tops in meters MSL
         ) = struct.unpack("<4xffBf", packet)
        '''
        values = data[5:]
        # Length of each RADR5 section in bytes: float (4 bytes) + float (4 bytes) + uint8 (1 byte) + float (4 bytes) = 13 bytes
        len_section = 13
        num_sections = int(len(values) / len_section)

        for i in range(num_sections):
          section_start = i * len_section
          section_data = values[section_start:section_start + len_section]
          lon, lat, storm_level, storm_height = struct.unpack("<ffBf", section_data)

          #print("lat:",lat, "lon", lon)

          if(lat != 0 and lon !=0):
            #print("lat:", lat, "lon:", lon)

            if(self.wxr_lonmax == None):
              self.wxr_lonmin = lon
              self.wxr_lonmax = lon
              self.wxr_latmin = lat
              self.wxr_latmax = lat

            if(lon > self.wxr_lonmax):
              self.wxr_lonmax = lon
            if(lon < self.wxr_lonmin):
              self.wxr_lonmin = lon
            if(lat > self.wxr_latmax):
              self.wxr_latmax = lat
            if(lat < self.wxr_latmin):
              self.wxr_latmin = lat
          
            self.wxr_map_temp[(lat,lon)] = storm_level


            if lon < self.wxr_lonmax and lat < self.wxr_latmax:
              # longitude spacing depending on latitude
              self.wxr_pixperlon = int(60.0 * math.cos(math.pi / 180.0 * (self.wxr_latmax + self.wxr_latmin) * 0.5))
              self.wxr_pixperlat = 60
              wxr_ncol = int((self.wxr_lonmax - self.wxr_lonmin) * self.wxr_pixperlon + 1)
              wxr_nlin = int((self.wxr_latmax - self.wxr_latmin) * self.wxr_pixperlat + 1)

              if wxr_ncol > self.wxr_ncol or wxr_nlin > self.wxr_nlin:
                self.wxr_ncol = wxr_ncol
                self.wxr_nlin = wxr_nlin
                print(f"Found WXR Lon/Lat Bounds: {self.wxr_lonmin} to {self.wxr_lonmax} / {self.wxr_latmin} to {self.wxr_latmax}, WXR Data size: {self.wxr_ncol} x {self.wxr_nlin}")
                print('keys', len(self.wxr_map_temp))
                self.wxr_map_temp = {}

      else:
        print("Unknown packet: ", binascii.hexlify(data))

      self.xplaneValues.update(retvalues)

    except:
      raise XPlaneTimeout
    return self.xplaneValues

  def FindIp(self):
    '''
    Find the IP of XPlane Host in Network.
    It takes the first one it can find. 
    '''
    self.BeaconData = {}
      
    # open socket for multicast group. 
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    if platform.system() == "Windows":
      sock.bind(('', self.MCAST_PORT))
    else:
      sock.bind((self.MCAST_GRP, self.MCAST_PORT))

    mreq = struct.pack("=4sl", socket.inet_aton(self.MCAST_GRP), socket.INADDR_ANY)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    sock.settimeout(3.0)
      
    # receive data
    try:   
      packet, sender = sock.recvfrom(1472)
      print("XPlane Beacon: ", packet.hex())

      # decode data
      # * Header
      header = packet[0:5]
      if header != b"BECN\x00":
        print("Unknown packet from "+sender[0])
        print(str(len(packet)) + " bytes")
        print(packet)
        print(binascii.hexlify(packet))
          
      else:
        # * Data
        data = packet[5:21]
        # struct becn_struct
        # {
        # 	uchar beacon_major_version;		// 1 at the time of X-Plane 10.40
        # 	uchar beacon_minor_version;		// 1 at the time of X-Plane 10.40
        # 	xint application_host_id;			// 1 for X-Plane, 2 for PlaneMaker
        # 	xint version_number;			// 104014 for X-Plane 10.40b14
        # 	uint role;						// 1 for master, 2 for extern visual, 3 for IOS
        # 	ushort port;					// port number X-Plane is listening on
        # 	xchr	computer_name[strDIM];		// the hostname of the computer 
        # };
        beacon_major_version = 0
        beacon_minor_version = 0
        application_host_id = 0
        xplane_version_number = 0
        role = 0
        port = 0
        (
          beacon_major_version,  # 1 at the time of X-Plane 10.40
          beacon_minor_version,  # 1 at the time of X-Plane 10.40
          application_host_id,   # 1 for X-Plane, 2 for PlaneMaker
          xplane_version_number, # 104014 for X-Plane 10.40b14
          role,                  # 1 for master, 2 for extern visual, 3 for IOS
          port,                  # port number X-Plane is listening on
          ) = struct.unpack("<BBiiIH", data)
        hostname = packet[21:-1] # the hostname of the computer
        hostname = hostname[0:hostname.find(0)]
        if beacon_major_version == 1 \
          and beacon_minor_version <= 2 \
          and application_host_id == 1:
          self.BeaconData["IP"] = sender[0]
          self.BeaconData["Port"] = port
          self.BeaconData["hostname"] = hostname.decode()
          self.BeaconData["XPlaneVersion"] = xplane_version_number
          self.BeaconData["role"] = role
          print("XPlane Beacon Version: {}.{}.{}".format(beacon_major_version, beacon_minor_version, application_host_id))
        else:
          print("XPlane Beacon Version not supported: {}.{}.{}".format(beacon_major_version, beacon_minor_version, application_host_id))
          raise XPlaneVersionNotSupported()

    except socket.timeout:
      print("XPlane IP not found.")
      raise XPlaneIpNotFound()
    finally:
      sock.close()

    return self.BeaconData

# Example how to use:
# You need a running xplane in your network. 
if __name__ == '__main__':

  longitude = "sim/flightmodel/position/longitude"
  latitude = "sim/flightmodel/position/latitude"
  mag_track = "sim/cockpit2/gauges/indicators/ground_track_mag_pilot"
  mag_var = "sim/flightmodel/position/magnetic_variation"

  xp = XPlaneUdp()

  try:
    beacon = xp.FindIp()
    print(beacon)
    print()
    
    xp.AddDataRef(longitude)
    xp.AddDataRef(latitude)
    xp.AddDataRef(mag_track)
    xp.AddDataRef(mag_var)

    xp.StartRadar(1000)

    '''
    # Load flight plan

    fpln = "I\n\
1100 Version\n\
CYCLE 2211\n\
ADEP KCLT\n\
DEPRWY RW36R\n\
SID KILNS4\n\
SIDTRANS KILNS\n\
ADES KEWR\n\
DESRWY RW22L\n\
STAR PHLBO3\n\
STARTRANS FAK\n\
APP I22L\n\
APPTRANS PATRN\n\
NUMENR 4\n\
1 KCLT ADEP 748.000000 35.213700 -80.949100\n\
11 AUDII DRCT 0.000000 36.202142 -78.810072\n\
3 FAK DRCT 0.000000 37.528508 -77.828219\n\
1 KEWR ADES 17.000000 40.692500 -74.168700"

    xp.SendFpl(fpln)
    '''

    '''
    # Send Command

    xp.SendCommand("sim/operation/reset_flight")
    '''

    while True:
      try:
        values = xp.GetValues()
        #print(values)

        latitude_value = values[latitude]
        longitude_value = values[longitude]
        mag_track_value = values[mag_track]
        mag_var_value = values[mag_var]

        map_heading_value = mag_track_value - mag_var_value

        #print("lat:", latitude_value, "long:", longitude_value, "heading:", map_heading_value)
        #print("wxr:", len(xp.weather_map))

      except XPlaneTimeout:
        print("XPlane Timeout")
        exit(0)

  except XPlaneVersionNotSupported:
    print("XPlane Version not supported.")
    exit(0)

  except XPlaneIpNotFound:
    print("XPlane IP not found. Probably there is no XPlane running in your local network.")
    exit(0)
