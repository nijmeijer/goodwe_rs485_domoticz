# Goodwe Python Plugin for Domoticz
#
# Author: Alex Nijmeijer
#
# Required Python Packages
#    pyserial, time
#
"""
<plugin key="Goodwe_RS485" name="Goodwe Solar Inverter via RS485" author="Alex Nijmeijer" version="1.0.5">
    <params>
        <param field="SerialPort" label="Serial Port" width="150px" required="true">
        </param>
        <param field="Mode6" label="Debug" width="150px">
            <options>
                <option label="None" value="0"  default="true" />
                <option label="Python Only" value="2"/>
                <option label="Basic Debugging" value="62"/>
                <option label="Basic+Messages" value="126"/>
                <option label="Queue" value="128"/>
                <option label="Connections Only" value="16"/>
                <option label="Connections+Queue" value="144"/>
                <option label="All" value="-1"/>
            </options>
        </param>
    </params>
</plugin>
"""

import time
import serial

#try:
import Domoticz
#except ImportError:
#  import fakeDomoticz as Domoticz

class CInverter :
  def __init__(self) :
    self.serialNumber=bytearray(b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00') # serial number (ascii) from inverter with zero appended
    self.address = 0                # address provided by this software
    self.addressConfirmed=False     # wether or not the address is confirmed by te inverter
    self.lastSeen = 0               # when was the inverter last seen? If not seen for 30 seconds the inverter is marked offline.
    self.isOnline = False           # is the inverter online (see above)
    self.isDTSeries = False         # is tri phase inverter (get phase 2, 3 info)
    self.vpv1=0.0
    self.vpv2=0.0
    self.ipv1=0.0
    self.ipv2 = 0.0
    self.vac1 = 0.0
    self.vac2 = 0.0
    self.vac3 = 0.0
    self.iac1 = 0.0
    self.iac2 = 0.0
    self.iac3 = 0.0
    self.fac1 = 0.0
    self.fac2 = 0.0
    self.fac3 = 0.0
    self.pac=0
    self.workMode=0
    self.temp = 0.0
    self.errorMessage=0
    self.eTotal=0
    self.hTotal=0
    self.tempFault = 0.0
    self.pv1Fault = 0.0
    self.pv2Fault = 0.0
    self.line1VFault = 0.0
    self.line2VFault = 0.0
    self.line3VFault = 0.0
    self.line1FFault = 0.0
    self.line2FFault = 0.0
    self.line3FFault = 0.0
    self.gcfiFault=0
    self.eDay = 0.0



class GoodWeCommunicator :
  def __init__(self) :
    self.inputBuffer = bytearray()
    self.debugPackets = False
    self.debugInverters = True
    self.GOODWE_COMMS_ADDRES = 0xFE
    self.DISCOVERY_INTERVAL_SECS = 60     # 10 secs between disovery
    self.PACKET_TIMEOUT_SECS = 5          # 5 sec packet timeout

    self.inverter = CInverter()

    #define INFO_INTERVAL 1000	          # get inverter info every second
    self.OFFLINE_TIMEOUT_SECS = 30        # 30 seconds no data -> inverter offline
    Domoticz.Log("Goodwe initialized")
    self.SerialStatus = False


  def serial_write_wrapper (self, btarr) :
    if (self.debugPackets) :
       strcat="Sending data to inverter(s): "
       for cnt in range(0,len(btarr)) :
        s=""
        if (cnt==0):
          s="Hdr: "
        if (cnt==2):
          s="SRC/DST: "
        if (cnt==4):
          s="Ctrl/Func: "
        if (cnt==6):
          s="Len: "
        if (cnt==7):
          s="Payld: "
        if (cnt==len(btarr)-2):
          s="CRC: "
        strcat=strcat+s+self.toHex(btarr[cnt])
       Domoticz.Log(strcat)
    try:
      self.goodweSerial.write(btarr)
    except:
      Domoticz.Log("Data sent failed")
      self.SerialStatus=False

  def connect(self, SerialPort) :
    Domoticz.Log("GoodWe Connect start.")
    self.goodweSerial = serial.Serial(port=SerialPort, baudrate=9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout = 0.1 )  # open serial port
    # remove all registered inverters. This is usefull when restarting the ESP. The inverter still thinks it is registered
    # but this program does not know the address. The timeout is 10 minutes.
    for cnt in range(1,2): #256
      self.sendRemoveRegistration(cnt)
      #time.sleep(1)
    Domoticz.Log("Goodwe Communicator connected")
    self.SerialStatus=True

  def start (self) :
    Domoticz.Log("GoodWe Communicator started.")

    self.lastDiscoverySentSeconds = 0
    self.lastReceivedByte = 0
    self.startPacketReceived = False
    self.numToRead = 0
    self.lastUsedAddress = 0
    self.ZeroSend = False

  def stop(self) :
    self.SerialStatus=False
    self.inverter.isOnline = False
    self.goodweSerial.close()             # close port

  def toHex(self,bt):
    s='{bt:02X} '.format(bt=bt)
    return s

  def sendRemoveRegistration(self, address) :
    #send out the remove address to the inverter. If the inverter is still connected it will reconnect after discovery
    self.sendData(address, 0x00, 0x02, bytearray())

  def sendData(self, address, controlCode, functionCode, data) :
    PacketData = bytearray()
    PacketData.append(0xAA)
    PacketData.append(0x55)
    PacketData.append(self.GOODWE_COMMS_ADDRES)
    PacketData.append(address)
    PacketData.append(controlCode)
    PacketData.append(functionCode)
    PacketData.append(len(data))

    # check if we need to write the data part and send it.
    if (len(data)>0) :
      PacketData += data

    # need to send out the crc which is the addition of all previous values.
    crc = 0
    for cnt in range(0,len(PacketData)) :
        crc += PacketData[cnt]
    PacketData.append((crc >> 8) & 0xff)  # MSB first
    PacketData.append(crc & 0xff)

    self.serial_write_wrapper(PacketData)
    return (len(PacketData))  #header, data, crc

  def sendDiscovery(self):
    # send out discovery for unregistered devices.
    if (self.debugInverters):
      Domoticz.Log("Sending discovery")
    self.sendData(0x7F, 0x00, 0x00, bytearray())

  def checkOfflineInverters(self) :
    # check inverter timeout
    # for (char index = 0; index < inverters.size(); ++index)
    newOnline = (time.time() - self.inverter.lastSeen) < self.OFFLINE_TIMEOUT_SECS
    #print ('newOnline', newOnline)
    if (self.inverter.isOnline and not(newOnline)) :
      # check if inverter timed out
      if (self.debugInverters) :
        s="Marking inverter @ address: {num) as offline".format(num=self.inverter.address)
        Domoticz.Log(s)
      self.inverter.isOnline = False
      self.inverter.addressConfirmed = False
      sendRemoveRegistration(self.inverter.address)  # send in case the inverter thinks we are online

    elif (not(self.inverter.isOnline) and not(newOnline)) : # still offline
      #offline inverter. Reset eday at midnight
      if (self.inverter.eDay > 0 and hour() == 0 and minute() == 0) :
        self.inverter.eDay = 0

      #check for data reset
      #if (inverters[index].vac1 > 0 and  millis() - inverter.lastSeen - OFFLINE_TIMEOUT > settingsManager->GetSettings()->inverterOfflineDataResetTimeout)
      #  #/reset all but eTotal, hTotal and eDay
      #  inverter.fac1        = inverter.fac2        = inverter.fac3        = inverter.gcfiFault   = \
      #  inverter.iac1        = inverter.iac2        = inverter.iac3        = inverter.ipv1        = inverter.ipv2        = \
      #  inverter.line1FFault = inverter.line1VFault = inverter.line2FFault = inverter.line2VFault = inverter.line3FFault = \
      #  inverter.line3VFault = inverter.pac         = inverter.pv1Fault    = inverter.pv2Fault    = inverter.vac1        = inverter.vac2 = \
      #  inverter.vac3        = inverter.vpv1        = inverter.vpv2        = inverter.temp        = 0;

    self.inverter.isOnline = newOnline

  def checkIncomingData(self) :
   time.sleep(0.1)
   try:
    if (self.goodweSerial.inWaiting()):
      if (self.debugPackets) :
        strcat='checkIncomingData: data received '

      while (self.goodweSerial.inWaiting() > 0) :
        incomingData = ord(self.goodweSerial.read(1))
        if (self.debugPackets) :
          strcat=strcat+'{bt:02X} '.format(bt=incomingData)

        # wait for packet start. if found read until data length  + data.
        # set the time we received the data so we can use some kind of timeout
        if (not(self.startPacketReceived) and (self.lastReceivedByte == 0xAA and incomingData == 0x55)) :
          # packet start received
          self.startPacketReceived = True
          self.lastReceived = time.time()
          self.inputBuffer=bytearray()
          self.numToRead = 0
          self.lastReceivedByte = 0x00 #reset last received for next packet
        elif (self.startPacketReceived) :
          if (self.numToRead > 0) or (len(self.inputBuffer)<5) :
             self.inputBuffer.append(incomingData)
             if (len(self.inputBuffer) == 5) :
               # we received the data length. keep on reading until data length is read.
               # we need to add two for the crc calculation
               self.numToRead = self.inputBuffer[4] + 2
             elif (len(self.inputBuffer)>5) :
                self.numToRead -= 1
          if (len(self.inputBuffer)>=5 and self.numToRead == 0) :
            # got the complete packet => parse it
            self.startPacketReceived = False
            self.parseIncomingData(self.inputBuffer)
        elif not(self.startPacketReceived) :
          self.lastReceivedByte = incomingData   # keep track of the last incoming byte so we detect the packet start

      if (self.debugPackets) :
        Domoticz.Log(strcat)

    if (self.startPacketReceived and (time.time() - self.lastReceived > PACKET_TIMEOUT_SECS)) :
      # there is an open packet timeout. 
      self.startPacketReceived = False #wait for start packet again
      Domoticz.Log("Warning: Comms timeout.")
   except :
      Domoticz.Log("Serialport failed, restart required")
      self.SerialStatus=False


  def parseIncomingData(self, incomingData):
    # first check the crc
    # Data always start without the start bytes of 0xAA 0x55
    # incomingDataLength also has the crc data in it

    crc = 0xAA + 0x55
    for cnt in range (0,  len(incomingData) - 2) :
      crc += incomingData[cnt]
    high = (crc >> 8) & 0xff
    low = crc & 0xff

    s=""
    if (self.debugPackets):
       s = "CRC received: "
       s = s + self.toHex(incomingData[len(incomingData) - 2])
       s = s + self.toHex(incomingData[len(incomingData) - 1])
       s = s + ", calculated CRC: "
       s = s + self.toHex(high) + " "
       s = s + self.toHex(low)
    # CRC matches?
    if not ((high == incomingData[len(incomingData) - 2]) and (low == incomingData[len(incomingData) - 1])) :
      s=s+"=> CRC Fail."
      Domoticz.Log(s)
      return
    if (self.debugPackets):
      Domoticz.Log(s+"=> CRC match.")

    # check the control code and function code to see what to do
    if (incomingData[2] == 0x00 and incomingData[3] == 0x80) :          #00 80 Register Request
       self.handleRegistration(incomingData[5:len(incomingData)-2])     #   -> payload (skip length)
    elif (incomingData[2] == 0x00 and incomingData[3] == 0x81) :        #00 81 Address Confirm
       self.handleRegistrationConfirmation(incomingData[0])             #   -> address
    elif (incomingData[2] == 0x01 and incomingData[3] == 0x81) :        #01 81 Resonse Running Info
      self.handleIncomingInformation(incomingData[0], incomingData[5:len(incomingData) - 2]) # + 5) payload


  def handleRegistration(self, serialNumber) :
    # check if the serialnumber isn't listed yet. If it is use that one
    # Add the serialnumber, generate an address and send it to the inverter
    if (len(serialNumber) != 16) :
      if (self.debugPackets):
       s="Error: length serialnumer != 16 ({num})".format(num=len(serialNumber))
       Domoticz.Log(s)
      return

    #for index in range (0,  index < inverters.size(); ++index)
    # check inverter
    if (self.inverter.serialNumber == serialNumber) :
      if (self.debugPackets):
        s="Already registered, inverter re-registered with address: {num}".format(num=self.inverter.address)
        Domoticz.Log(s)
      # found it. Set to unconfirmed and send out the existing address to the inverter
      self.inverter.addressConfirmed = False
      self.inverter.lastSeen = time.time()
      self.sendAllocateRegisterAddress(serialNumber, self.inverter.address)
      return

    # still here. This a new inverter
    # GoodWeCommunicator::GoodweInverterInformation newInverter
    # newInverter.addressConfirmed = false;
    self.inverter.addressConfirmed = False
    # newInverter.lastSeen = millis();
    self.inverter.lastSeen = time.time()
    # newInverter.isDTSeries = false; //TODO. Determine if DT series inverter by getting info
    self.inverter.serialNumer=serialNumber
    # //get the new address. Add one (overflows at 255) and check if not in use
    self.lastUsedAddress += 1
    #while (getInverterInfoByAddress(lastUsedAddress) != nullptr)
    #  self.lastUsedAddress += 1
    # newInverter.address = lastUsedAddress;
    self.inverter.address = self.lastUsedAddress
    # inverters.push_back(newInverter);
    #    if (self.debugInverters) :
    #  s="New inverter found. Current # registrations: {num}".format(num=1)
    #  Domoticz.Log(s)

    self.sendAllocateRegisterAddress(serialNumber, self.lastUsedAddress)


  def handleRegistrationConfirmation(self, address) :
    if (self.debugInverters) :
      s="Handling registration information for address: {num}".format(num=address)
      Domoticz.Log(s)
    # lookup the inverter and set it to confirmed
    self.inverter = self.getInverterInfoByAddress(address)
    if (self.inverter) :
      if (self.debugInverters) :
         Domoticz.Log("Inverter information found in list of inverters.")
      self.inverter.addressConfirmed = True
      self.inverter.isOnline = False # //inverter is online, but we first need to get its information
      self.inverter.lastSeen = time.time() #millis()
    else :
      if (self.debugInverters) :
        s="Error. Could not find the inverter with address: {num}".format(num=address)
        Domoticz.Log(s)
        #s="Current # registrations: {num}".format(num=1)     # (inverters.size())
        #Domoticz.Log(s)
        # get the information straight away
      self.askInverterForInformation(address)

  def handleIncomingInformation(self, address, data) :
    #Domoticz.Log("handleIncomingInformation")
    # //need to parse the information and update our struct
    # //parse all pairs of two bytes and output them
    # auto inverter = `erterInfoByAddress(address);
    # if (inverter == nullptr) return;
    inverter = self.inverter

    if (len(data) < 44): #//minimum for non dt series
      Domoticz.Log("handleIncomingInformation: Error, Insuffient information")
      return

    # data from iniverter, means online
    inverter.lastSeen = time.time()

    dtPtr = 0
    inverter.vpv1 = self.bytesToFloat(data[dtPtr:(dtPtr+2)],10)
    dtPtr += 2
    inverter.vpv2 = self.bytesToFloat(data[dtPtr:(dtPtr+2)], 10)
    dtPtr += 2
    inverter.ipv1 = self.bytesToFloat(data[dtPtr:(dtPtr+2)], 10)
    dtPtr += 2
    inverter.ipv2 = self.bytesToFloat(data[dtPtr:(dtPtr+2)], 10)
    dtPtr += 2
    inverter.vac1 = self.bytesToFloat(data[dtPtr:(dtPtr+2)], 10)
    dtPtr += 2
    if (inverter.isDTSeries) :
      inverter.vac2 = self.bytesToFloat(data[dtPtr:(dtPtr+2)], 10)
      dtPtr += 2
      inverter.vac3 = self.bytesToFloat(data[dtPtr:(dtPtr+2)], 10)
      dtPtr += 2
    inverter.iac1 = self.bytesToFloat(data[dtPtr:(dtPtr+2)], 10)
    dtPtr += 2
    if (inverter.isDTSeries) :
      inverter.iac2 = self.bytesToFloat(data[dtPtr:(dtPtr+2)], 10)
      dtPtr += 2
      inverter.iac3 = self.bytesToFloat(data[dtPtr:(dtPtr+2)], 10)
      dtPtr += 2
    inverter.fac1 = self.bytesToFloat(data[dtPtr:(dtPtr+2)], 100)
    dtPtr += 2
    if (inverter.isDTSeries):
      inverter.fac2 = self.bytesToFloat(data[dtPtr:(dtPtr+2)], 100)
      dtPtr += 2
      inverter.fac3 = self.bytesToFloat(data[dtPtr:(dtPtr+2)], 100)
      dtPtr += 2
    inverter.pac = ((data[dtPtr]) << 8) | (data[dtPtr + 1])
    dtPtr += 2
    inverter.workMode = ((data[dtPtr]) << 8) | (data[dtPtr + 1])
    dtPtr += 2
    inverter.temp = self.bytesToFloat(data[dtPtr:dtPtr+2], 10)
    dtPtr += 2+4
    inverter.eTotal = self.fourBytesToFloat(data[dtPtr:dtPtr+4], 10)
    dtPtr += 4
    inverter.hTotal = self.fourBytesToFloat(data[dtPtr:dtPtr+4], 1)
    dtPtr += 4+8
    if (inverter.isDTSeries):
      dtPtr += 4
    dtPtr += 2
    if (inverter.isDTSeries):
      dtPtr += 4
    dtPtr += 2
    inverter.eDay   = self.bytesToFloat(data[dtPtr: dtPtr+2], 10)

    # isonline is set after first batch of data is set so readers get actual data
    # some sanity checks:
    if (inverter.eTotal>1) and (inverter.workMode>0) :
      inverter.isOnline = True
    else :
      inverter.isOnline = False


    if (self.debugInverters) :
      s=  'Pac={num:.1f} '.format(num=inverter.pac)
      s=s+'Eday={num:.1f} '.format(num=inverter.eDay)
      s=s+'V1={num:.1f} '.format(num=inverter.vpv1)
      s=s+'V2={num:.1f} '.format(num=inverter.vpv2)
      s=s+'I1={num:.1f} '.format(num=inverter.ipv1)
      s=s+'I2={num:.1f} '.format(num=inverter.ipv2)
      s=s+'Fac={num:.1f} '.format(num=inverter.fac1)
      s=s+'eTot={num:.1f} '.format(num=inverter.eTotal)
      s=s+'hTot={num:.1f} '.format(num=inverter.hTotal)
      s=s+'T={num:.1f} '.format(num=inverter.temp)
      s=s+'Mode={num} '.format(num=inverter.workMode)
      Domoticz.Log(s)

  def bytesToFloat(self, bt, factor) :
    # Convert two bytes to float by converting to short and then dividing it by factor
    return  ((bt[0] << 8) | bt[1]) / factor

  def fourBytesToFloat(self, bt, factor) :
    # Convert four bytes to float by converting to int32 and then dividing it by factor
    return  ((bt[0] << 24) | (bt[1] << 16)|  (bt[2] << 8) | bt[3]) / factor

  def askAllInvertersForInformation(self) :
    #for (char index = 0; index < inverters.size(); ++index) :
      if ((self.inverter.addressConfirmed) and (self.inverter.isOnline))  :
         self.askInverterForInformation(self.inverter.address)
      else :
        if (self.debugInverters) :
          s=  "Not asking inverter with address: {num} ".format(num=self.inverter.address)
          s=s+"for information. Addressconfirmed: {num}".format(num=self.inverter.addressConfirmed)
          s=s+", isOnline: {num}".format(num=self.inverter.isOnline)
          Domoticz.Log(s)


  def askInverterForInformation(self, address) :
    self.sendData(address, 0x01, 0x01, bytearray())

  def getInverterInfoByAddress(self, address) :
    #for chr in range(1, 1) # inverters.size())
    # check inverter 
    #if (inverters[index].address == address)
    #  return &inverters[index]
    return self.inverter
    #return 0

  def sendAllocateRegisterAddress(self, serialNumber, address) :
    if (self.debugInverters) :
      s="SendAllocateRegisterAddress address: {num}".format(num=address)
      Domoticz.Log(s)

    # create our registrationpacket with serialnumber and address and send it over
    # char RegisterData[17];
    # memcpy(RegisterData, serialNumber, 16);
    RegisterData=serialNumber
    RegisterData += address.to_bytes(1,'big')   #append bytearray (1).to_bytes(1, byteorder='big')
    # need to send alloc msg
    self.sendData(0x7F, 0x00, 0x01, RegisterData)


  def handle(self) :
    # always check for incoming data
    self.checkIncomingData()

    # check for offline inverters
    self.checkOfflineInverters()

    # discovery every 10 secs.
    if (time.time() - self.lastDiscoverySentSeconds >= self.DISCOVERY_INTERVAL_SECS) :
      self.sendDiscovery()
      self.lastDiscoverySentSeconds = time.time()

    # ask for info update every second
    if (time.time() - self.lastDiscoverySentSeconds >= 1) :
      self.askAllInvertersForInformation()
      self.lastDiscoverySentSeconds = time.time()
    self.checkIncomingData()




class BasePlugin:
    enabled = False
    def __init__(self):
        Domoticz.Log("__init__ called")
        self.Goodwe = GoodWeCommunicator()
        return

    def onStart(self):
        Domoticz.Log("BasePlugin onStart called")
        if Parameters["Mode6"] != "0":
           Domoticz.Debugging((Parameters["Mode6"]))
           #Domoticz.Debugging(int(Parameters["Mode6"]))
           DumpConfigToLog()

        if (len(Devices) == 0 ):
          Domoticz.Log("Adding devices.")
          Domoticz.Device("Solar Power", Unit=1, Type=243, Subtype=29).Create()

        Domoticz.Log("Plugin is started @ Serial " + Parameters["SerialPort"])

        self.Goodwe.start()
        Domoticz.Log("Goodwe start called")
        self.Goodwe.connect(Parameters["SerialPort"])
        Domoticz.Log("Goodwe connect called") 

        Domoticz.Heartbeat(5)  # poll inverter every 2 seconds

    def onStop(self):
        Domoticz.Log("BasePlugin onStop called")
        self.Goodwe.stop()

    def onConnect(self, Connection, Status, Description):
        Domoticz.Log("onConnect called")

    def onMessage(self, Connection, Data):
        Domoticz.Log("onMessage called")

    def onCommand(self, DeviceID, Unit, Command, Level, Color):
        Domoticz.Log("onCommand called for Device " + str(DeviceID) + " Unit " + str(Unit) + ": Parameter '" + str(Command) + "', Level: " + str(Level))

    def onNotification(self, Name, Subject, Text, Status, Priority, Sound, ImageFile):
        Domoticz.Log("Notification: " + Name + "," + Subject + "," + Text + "," + Status + "," + str(Priority) + "," + Sound + "," + ImageFile)

    def onDisconnect(self, Connection):
        Domoticz.Log("onDisconnect called")

    def onHeartbeat(self):
        # Domoticz.Log("BasePlugin onHeartbeat called")
        try:
          if self.Goodwe.SerialStatus==False :
            Domoticz.Log("Serialport closed, restarting...")
            try:
              onStop()
            except:
              pass
            onStart()

          self.Goodwe.handle()
          # Domoticz.Log("Goodwe handle exited ok")
          try:
            if (self.Goodwe.inverter.isOnline) and (self.Goodwe.inverter.eTotal > 1) :
              goodwe_pv_watt   = self.Goodwe.inverter.pac
              goodwe_wh_total  = self.Goodwe.inverter.eTotal*1000
              #Domoticz.Log("Goodwe: " + str(goodwe_wh_total) + "; " + str(goodwe_pv_watt) )
              Devices[1].Update(nValue=goodwe_pv_watt, sValue=(str(goodwe_pv_watt) + "; " +str(goodwe_wh_total)) )
              self.ZeroSend=False
          except:
            Domoticz.Log("Error getting info from Goodwe inverter on " + Parameters["SerialPort"] + "Not returning anything")

        except:
          Domoticz.Log("Goodwe handle failed")
          self.Goodwe.SerialStatus=False
          if  (self.Goodwe.inverter.eTotal > 1)  and (self.ZeroSend==False) :
              goodwe_wh_total  = self.Goodwe.inverter.eTotal*1000
              Devices[1].Update(nValue=0, sValue=(str(0) + "; " +str(goodwe_wh_total)) )
              self.ZeroSend=True


global _plugin
_plugin = BasePlugin()

def onStart():
    global _plugin
    _plugin.onStart()

def onStop():
    global _plugin
    _plugin.onStop()

def onConnect(Connection, Status, Description):
    global _plugin
    _plugin.onConnect(Connection, Status, Description)

def onMessage(Connection, Data):
    global _plugin
    _plugin.onMessage(Connection, Data)

def onCommand(DeviceID, Unit, Command, Level, Color):
    global _plugin
    _plugin.onCommand(DeviceID, Unit, Command, Level, Color)

def onNotification(Name, Subject, Text, Status, Priority, Sound, ImageFile):
    global _plugin
    _plugin.onNotification(Name, Subject, Text, Status, Priority, Sound, ImageFile)

def onDisconnect(Connection):
    global _plugin
    _plugin.onDisconnect(Connection)

def onHeartbeat():
    global _plugin
    _plugin.onHeartbeat()

# Generic helper functions
def DumpConfigToLog():
    Domoticz.Debug("Parameters:")
    for x in Parameters:
        if Parameters[x] != "":
            Domoticz.Debug( "'" + x + "':'" + str(Parameters[x]) + "'")
    Domoticz.Debug("Devices:")
    Domoticz.Debug("Device count: " + str(len(Devices)))
    for DeviceName in Devices:
        Device = Devices[DeviceName]
        Domoticz.Debug("Device ID:       '" + str(Device.DeviceID) + "'")
        #Domoticz.Debug("--->Unit Count:      '" + str(len(Device.Units)) + "'")
        #for UnitNo in Device.Units:
        #    Unit = Device.Units[UnitNo]
        #    Domoticz.Debug("--->Unit:           " + str(UnitNo))
        #    Domoticz.Debug("--->Unit Name:     '" + Unit.Name + "'")
        #    Domoticz.Debug("--->Unit nValue:    " + str(Unit.nValue))
        #    Domoticz.Debug("--->Unit sValue:   '" + Unit.sValue + "'")
        #    Domoticz.Debug("--->Unit LastLevel: " + str(Unit.LastLevel))
    return
