#This program uses some previous code written for the Omnitool which has accelerometer
# functions in addition to controlling the BME. Instead of writing from scratch, importated
# that code containing some relavent functions with room to expand (like with display or buzzer).
import numpy as np
import smbus
import time
from time import sleep
from ctypes import c_short 
from ctypes import c_byte
from ctypes import c_ubyte
from datetime import datetime
import Adafruit_SSD1306
from PIL import Image,ImageDraw, ImageFont
import RPi.GPIO as GPIO
import os

GPIO.setmode(GPIO.BCM)
laser = 38 #not relavent yet
buzzer = 40
GPIO.setup(laser, GPIO.OUT)
GPIO.output(laser, GPIO.LOW)
GPIO.setup(buzzer, GPIO.OUT)
GPIO.output(buzzer, GPIO.LOW)

class mpu6050:
  # Global Variables
  GRAVITIY_MS2 = 9.80665
  address = None
  bus = None

  # Scale Modifiers
  ACCEL_SCALE_MODIFIER_2G = 16384.0
  ACCEL_SCALE_MODIFIER_4G = 8192.0
  ACCEL_SCALE_MODIFIER_8G = 4096.0
  ACCEL_SCALE_MODIFIER_16G = 2048.0

  GYRO_SCALE_MODIFIER_250DEG = 131.0
  GYRO_SCALE_MODIFIER_500DEG = 65.5
  GYRO_SCALE_MODIFIER_1000DEG = 32.8
  GYRO_SCALE_MODIFIER_2000DEG = 16.4

  # Pre-defined ranges
  ACCEL_RANGE_2G = 0x00
  ACCEL_RANGE_4G = 0x08
  ACCEL_RANGE_8G = 0x10
  ACCEL_RANGE_16G = 0x18

  GYRO_RANGE_250DEG = 0x00
  GYRO_RANGE_500DEG = 0x08
  GYRO_RANGE_1000DEG = 0x10
  GYRO_RANGE_2000DEG = 0x18

  # MPU-6050 Registers
  PWR_MGMT_1 = 0x6B
  PWR_MGMT_2 = 0x6C

  ACCEL_XOUT0 = 0x3B
  ACCEL_YOUT0 = 0x3D
  ACCEL_ZOUT0 = 0x3F

  TEMP_OUT0 = 0x41

  GYRO_XOUT0 = 0x43
  GYRO_YOUT0 = 0x45
  GYRO_ZOUT0 = 0x47

  ACCEL_CONFIG = 0x1C
  GYRO_CONFIG = 0x1B

  def __init__(self, address, bus=1):
    self.address = address
    self.bus = smbus.SMBus(bus)
    # Wake up the MPU-6050 since it starts in sleep mode
    self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)

  # I2C communication methods

  def read_i2c_word(self, register):
    """Read two i2c registers and combine them.

    register -- the first register to read from.
    Returns the combined read results.
    """
    # Read the data from the registers
    high = self.bus.read_byte_data(self.address, register)
    low = self.bus.read_byte_data(self.address, register + 1)

    value = (high << 8) + low

    if (value >= 0x8000):
      return -((65535 - value) + 1)
    else:
      return value

  # MPU-6050 Methods

  def get_temp(self):
    """Reads the temperature from the onboard temperature sensor of the MPU-6050.

    Returns the temperature in degrees Celcius.
    """
    raw_temp = self.read_i2c_word(self.TEMP_OUT0)

    # Get the actual temperature using the formule given in the
    # MPU-6050 Register Map and Descriptions revision 4.2, page 30
    actual_temp = (raw_temp / 340.0) + 36.53

    return actual_temp

  def set_accel_range(self, accel_range):
    """Sets the range of the accelerometer to range.

    accel_range -- the range to set the accelerometer to. Using a
    pre-defined range is advised.
    """
    # First change it to 0x00 to make sure we write the correct value later
    self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)

    # Write the new range to the ACCEL_CONFIG register
    self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, accel_range)

  def read_accel_range(self, raw=False):
    """Reads the range the accelerometer is set to.

    If raw is True, it will return the raw value from the ACCEL_CONFIG
    register
    If raw is False, it will return an integer: -1, 2, 4, 8 or 16. When it
    returns -1 something went wrong.
    """
    raw_data = self.bus.read_byte_data(self.address, self.ACCEL_CONFIG)

    if raw is True:
      return raw_data
    elif raw is False:
      if raw_data == self.ACCEL_RANGE_2G:
        return 2
      elif raw_data == self.ACCEL_RANGE_4G:
        return 4
      elif raw_data == self.ACCEL_RANGE_8G:
        return 8
      elif raw_data == self.ACCEL_RANGE_16G:
        return 16
      else:
        return -1

  def get_accel_data(self, g=False):
    """Gets and returns the X, Y and Z values from the accelerometer.

    If g is True, it will return the data in g
    If g is False, it will return the data in m/s^2
    Returns a dictionary with the measurement results.
    """
    x = self.read_i2c_word(self.ACCEL_XOUT0)
    y = self.read_i2c_word(self.ACCEL_YOUT0)
    z = self.read_i2c_word(self.ACCEL_ZOUT0)

    accel_scale_modifier = None
    accel_range = self.read_accel_range(True)

    if accel_range == self.ACCEL_RANGE_2G:
      accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G
    elif accel_range == self.ACCEL_RANGE_4G:
      accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_4G
    elif accel_range == self.ACCEL_RANGE_8G:
      accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_8G
    elif accel_range == self.ACCEL_RANGE_16G:
      accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_16G
    else:
      print("Unkown range - accel_scale_modifier set to self.ACCEL_SCALE_MODIFIER_2G")
      accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G

    x = x / accel_scale_modifier
    y = y / accel_scale_modifier
    z = z / accel_scale_modifier

    if g is True:
      return {'x': x, 'y': y, 'z': z}
    elif g is False:
      x = x * self.GRAVITIY_MS2
      y = y * self.GRAVITIY_MS2
      z = z * self.GRAVITIY_MS2
      return {'x': x, 'y': y, 'z': z}

  def set_gyro_range(self, gyro_range):
    """Sets the range of the gyroscope to range.

    gyro_range -- the range to set the gyroscope to. Using a pre-defined
    range is advised.
    """
    # First change it to 0x00 to make sure we write the correct value later
    self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)

    # Write the new range to the ACCEL_CONFIG register
    self.bus.write_byte_data(self.address, self.GYRO_CONFIG, gyro_range)

  def read_gyro_range(self, raw=False):
    """Reads the range the gyroscope is set to.

    If raw is True, it will return the raw value from the GYRO_CONFIG
    register.
    If raw is False, it will return 250, 500, 1000, 2000 or -1. If the
    returned value is equal to -1 something went wrong.
    """
    raw_data = self.bus.read_byte_data(self.address, self.GYRO_CONFIG)

    if raw is True:
      return raw_data
    elif raw is False:
      if raw_data == self.GYRO_RANGE_250DEG:
        return 250
      elif raw_data == self.GYRO_RANGE_500DEG:
        return 500
      elif raw_data == self.GYRO_RANGE_1000DEG:
        return 1000
      elif raw_data == self.GYRO_RANGE_2000DEG:
        return 2000
      else:
        return -1

  def get_gyro_data(self, gx, gy, gz):
    """Gets and returns the X, Y and Z values from the gyroscope.

    Returns the read values in a dictionary.
    """
    x = self.read_i2c_word(self.GYRO_XOUT0)
    y = self.read_i2c_word(self.GYRO_YOUT0)
    z = self.read_i2c_word(self.GYRO_ZOUT0)

    gyro_scale_modifier = None
    gyro_range = self.read_gyro_range(True)

    if gyro_range == self.GYRO_RANGE_250DEG:
      gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG
    elif gyro_range == self.GYRO_RANGE_500DEG:
      gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_500DEG
    elif gyro_range == self.GYRO_RANGE_1000DEG:
      gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_1000DEG
    elif gyro_range == self.GYRO_RANGE_2000DEG:
      gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_2000DEG
    else:
      print("Unkown range - gyro_scale_modifier set to self.GYRO_SCALE_MODIFIER_250DEG")
      gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG

    x = x / gyro_scale_modifier
    y = y / gyro_scale_modifier
    z = z / gyro_scale_modifier

    if gx == True:
      return {'x': x}
    if gy == True:
      return {'y': y}
    if gz == True:
      return {'z': z}

  def get_all_data(self):
    """Reads and returns all the available data."""
    temp = self.get_temp()
    accel = self.get_accel_data()
    gyro = self.get_gyro_data()

    return [accel, gyro, temp]

addr = 0x76 # Default device I2C address

bus = smbus.SMBus(1) # Rev 2 Pi, Pi 2 & Pi 3 uses bus 1
                     # Rev 1 Pi uses bus 0
time.sleep(1)


def getShort(data, index):
  # return two bytes from data as a signed 16-bit value
  return c_short((data[index + 1] << 8) + data[index]).value

def getUShort(data, index):
    # return two bytes from data as an unsigned 16-bit value
    return (data[index + 1] << 8) + data[index]


def getChar(data, index):
  # return one byte from data as a signed char
  result = data[index]
  if result > 127:
    result -= 256
  return result

def getUChar(data, index):
    # return one byte from data as an unsigned char
    result = data[index] & 0xFF
    return result


def readBME280ID(addr=0x76):
  # Chip ID Register Address
  REG_ID = 0xD0
  (chip_id, chip_version) = bus.read_i2c_block_data(addr, REG_ID, 2)
  return (chip_id, chip_version)


def readBME280All(addr=0X76):
  # Register Addresses
  REG_DATA = 0xF7
  REG_CONTROL = 0xF4
  REG_CONFIG = 0xF5

  REG_CONTROL_HUM = 0xF2
  REG_HUM_MSB = 0xFD
  REG_HUM_LSB = 0xFE

  # Oversample setting - page 27
  OVERSAMPLE_TEMP = 2
  OVERSAMPLE_PRES = 2
  MODE = 1

  # Oversample setting for humidity register - page 26
  OVERSAMPLE_HUM = 2
  bus.write_byte_data(addr, REG_CONTROL_HUM, OVERSAMPLE_HUM)

  control = OVERSAMPLE_TEMP << 5 | OVERSAMPLE_PRES << 2 | MODE
  bus.write_byte_data(addr, REG_CONTROL, control)

  # Read blocks of calibration data from EEPROM
  # See Page 22 data sheet
  cal1 = bus.read_i2c_block_data(addr, 0x88, 24)
  cal2 = bus.read_i2c_block_data(addr, 0xA1, 1)
  cal3 = bus.read_i2c_block_data(addr, 0xE1, 7)

  # Convert byte data to word values
  dig_T1 = getUShort(cal1, 0)
  dig_T2 = getShort(cal1, 2)
  dig_T3 = getShort(cal1, 4)

  dig_P1 = getUShort(cal1, 6)
  dig_P2 = getShort(cal1, 8)
  dig_P3 = getShort(cal1, 10)
  dig_P4 = getShort(cal1, 12)
  dig_P5 = getShort(cal1, 14)
  dig_P6 = getShort(cal1, 16)
  dig_P7 = getShort(cal1, 18)
  dig_P8 = getShort(cal1, 20)
  dig_P9 = getShort(cal1, 22)

  dig_H1 = getUChar(cal2, 0)
  dig_H2 = getShort(cal3, 0)
  dig_H3 = getUChar(cal3, 2)

  dig_H4 = getChar(cal3, 3)
  dig_H4 = (dig_H4 << 24) >> 20
  dig_H4 = dig_H4 | (getChar(cal3, 4) & 0x0F)

  dig_H5 = getChar(cal3, 5)
  dig_H5 = (dig_H5 << 24) >> 20
  dig_H5 = dig_H5 | (getUChar(cal3, 4) >> 4 & 0x0F)

  dig_H6 = getChar(cal3, 6)

  # Wait in ms (Datasheet Appendix B: Measurement time and current calculation)
  wait_time = 1.25 + (2.3 * OVERSAMPLE_TEMP) + ((2.3 * OVERSAMPLE_PRES) + 0.575) + ((2.3 * OVERSAMPLE_HUM) + 0.575)
  time.sleep(wait_time / 1000)  # Wait the required time

  # Read temperature/pressure/humidity
  data = bus.read_i2c_block_data(addr, REG_DATA, 8)
  pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
  temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
  hum_raw = (data[6] << 8) | data[7]

  # Refine temperature
  var1 = ((((temp_raw >> 3) - (dig_T1 << 1))) * (dig_T2)) >> 11
  var2 = (((((temp_raw >> 4) - (dig_T1)) * ((temp_raw >> 4) - (dig_T1))) >> 12) * (dig_T3)) >> 14
  t_fine = var1 + var2
  temperature = float(((t_fine * 5) + 128) >> 8);

  # Refine pressure and adjust for temperature
  var1 = t_fine / 2.0 - 64000.0
  var2 = var1 * var1 * dig_P6 / 32768.0
  var2 = var2 + var1 * dig_P5 * 2.0
  var2 = var2 / 4.0 + dig_P4 * 65536.0
  var1 = (dig_P3 * var1 * var1 / 524288.0 + dig_P2 * var1) / 524288.0
  var1 = (1.0 + var1 / 32768.0) * dig_P1
  if var1 == 0:
    pressure = 0
  else:
    pressure = 1048576.0 - pres_raw
    pressure = ((pressure - var2 / 4096.0) * 6250.0) / var1
    var1 = dig_P9 * pressure * pressure / 2147483648.0
    var2 = pressure * dig_P8 / 32768.0
    pressure = pressure + (var1 + var2 + dig_P7) / 16.0

    # Refine humidity
  humidity = t_fine - 76800.0
  humidity = (hum_raw - (dig_H4 * 64.0 + dig_H5 / 16384.0 * humidity)) * (
          dig_H2 / 65536.0 * (1.0 + dig_H6 / 67108864.0 * humidity * (1.0 + dig_H3 / 67108864.0 * humidity)))
  humidity = humidity * (1.0 - dig_H1 * humidity / 524288.0)
  if humidity > 100:
    humidity = 100
  elif humidity < 0:
    humidity = 0

  return temperature / 100.0, pressure / 100.0, humidity


def temp():
  # (chip_id, chip_version) = readBME280ID()
  # print("Chip ID     :", chip_id)
  # print("Version     :", chip_version)

  temperature, pressure, humidity = readBME280All()
  print("Temperature : ", temperature, "C")
  print("Pressure : ", pressure, "hPa")
  print("Humidity : ", humidity, "%")
  # file = open("070921.txt", "a")
  # file.write("Temp={0:0.1f}*C Humidity={1:0.1f}%".format(temperature, humidity))
  # file.write(datetime.today().strftime('%Y-%m-%d %H:%M:%S') + "\n")
  # file.close()
  # time.sleep(5)
  # if temperature > 27:
  #  print("Temp control activate")

def accel(ax, ay, az):
  caliaccelfactorx = -0.05
  caliaccelfactory = 0.2
  caliaccelfactorz = -0.4
  #print("Temp: ", mpu.get_temp(), "C")
  array = []
  try:
    accel_data = mpu.get_accel_data()
  except:
    pass
  if ax == True:
    #print("Accel X:", accel_data['x'] + caliaccelfactorx)
    array.append(accel_data['x'])
  else:
    pass
  if ay == True:
    #print("Accel Y:", accel_data['y'] + caliaccelfactory)
    array.append(accel_data['y'])
  else:
    pass
  if az == True:
    #print("Accel Z:", accel_data['z'] + caliaccelfactorz)
    array.append(accel_data['z'])
  else:
    pass
  #print(array)
  return array
  #print("Accel X:", accel_data['x'] + caliaccelfactorx)
  #print("Accel Y:", accel_data['y'] + caliaccelfactory)
  #gyro_data = mpu.get_gyro_data()
  #print("X Axis:", gyro_data['x'] + caligyrofactorx, " Deg/S")
  #print("Y Axis:", gyro_data['y'] + caligyrofactory, " Deg/S")
  #print("Z Axis:", gyro_data['z'] + caligyrofactorz, " Deg/S")
#  if (accel_data['z']) > 9:
#    return True
#  else:
#    return False

def gyro(gx, gy, gz):
  caligyrofactorx = 1.6
  caligyrofactory = 2.8
  caligyrofactorz = 2.2

def display():
  display = Adafruit_SSD1306.SSD1306_128_32(rst=None)
  display.begin()
  display.clear()
  display.display()
  displayWidth = display.width
  displayHeight = display.height
  image = Image.new('1', (displayWidth, displayHeight))
  draw = ImageDraw.Draw(image)
  font = ImageFont.load_default()
  draw.text((0, 0), "Initializing...", font=font, fill=255)
  display.image(image)
  display.display()
  sleep(2)
  display.clear()
  display.display()
  sleep(1)
  draw.rectangle((0, 0, displayWidth, displayHeight), outline=0, fill=0)
  draw.text((0, 16), "Awaiting Input", font=font, fill=255)
  display.image(image)
  display.display()
  print("Complete")


def chamber(state):
  print("Chambering")
  xa_array = []
  ya_array = []
  za_array = []
  xg_array = []
  yg_array = []
  zg_array = []
  sense_xa = []
  sense_ya = []
  sense_za = []
  sense_xg = []
  sense_yg = []
  sense_zg = []
  for i in range(10):
    try:
      orient_check = [accel(True, False, True)]
      rot_check = [gyro(False, True, False)]
      xa_array.append(orient_check[0])
      ya_array.append(orient_check[1])
      za_array.append(orient_check[2])
      xg_array.append(rot_check[0])
      yg_array.append(rot_check[1])
      zg_array.append(rot_check[2])
    except:
      pass
  # start time clock and integer value to take data points for a period of time
  start = time.time()
  while (time.time() - start < 5):
    if state == True:
      for i in range(10):  # 10 data points for moving average
        try:
          orient_check = [accel(True, False, True)]
        except:
          pass
          # print(orient_check[0])
          # print(x_array)
          xa_array[i] = orient_check[0]
          ya_array[i] = orient_check[1]
          za_array[i] = orient_check[2]
          xg_array[i] = rot_check[0]
          yg_array[i] = rot_check[1]
          zg_array[i] = rot_check[2]
          # y_array.append(orient_check[1])
          # z_array.append(orient_check[2])
          if len(xa_array) > 9:
            sense_xa = np.mean(xa_array)
            sense_ya = np.mean(ya_array)
            sense_za = np.mean(za_array)
            sense_xg = np.mean(xg_array)
            sense_yg = np.mean(yg_array)
            sense_zg = np.mean(zg_array)
            print(sense_xa)
            print(sense_ya)
            print(sense_za)
            print(sense_yg)
            if sense_y > 10 and sense_z < -8:
              print("Punch Start!")
              sleep(1)
            if sense_z > 7 and sense_y < -12:
              print("Punch End!")
              sleep(1)
    # maxim = max(sense_x)
    # print(maxim)
    # if maxim > 9:
    # print("Punch!")
    # print(x_array, y_array, z_array)


def moving_average(a, n):
  ret = np.cumsum(a, dtype=float)
  ret[n:] = ret[n:] - ret[:-n]
  return ret[n-1:] / n


mpu = mpu6050(0x68)
mpu.set_gyro_range(0X08)
mpu.set_accel_range(0X08)

if __name__ == "__main__":
  while True:
    try:
      # GPIO.output(laser, GPIO.LOW)
      # os.system('clear')
      [temp_pull_1, hum_pull_1, press_pull_1]  = readBME280All() #BME temperature sensor
      temp_pull_2 = get_temp()
      filename = datetime.today().strftime("%Y%m%d")
      file = open(filename + ".txt", "a")
      file.write("Temp1={0:0.01f}* Temp2={1:0.01f}* Hum={2:0.01f}* Press={3.01f}".format(temp_pull_1, temp_pull_2, hum_pull_1, press_pull_1)
      file.write(datetime.today().strftime('%H%M%S') + "\n")
      file.close()
      sleep(2))
      # data_pull = accel(True, True, True)
      #print(f"Ax = {data_pull[0]:.2f} m/s^2\nAy = {data_pull[1]:.2f} m/s^2\nAz = {data_pull[2]:.2f} m/s^2\n")
      #chamber_val = (data_pull[2])
      #if chamber_val < -9:
      #  print("Chamber State Active")
      #  chamber(True)
    except:
      pass
    #  chamber(True)
    #  sleep(2)
    # if chamber_state == False:
    #  print("Inactive")
    #sleep(0.1)
    #temp()
    #display()
    #GPIO.output(laser, GPIO.HIGH)
    


#
# if __name__ == "__main__":
#
#     file = open("Gyro.txt", "a")
#     file.write("XAccel={0:0.01f}* YAccel={1:0.01f}*".format(temperature, humidity))
#     file.write(datetime.today().strftime('%S') + "\n")
#     file.close()

   
