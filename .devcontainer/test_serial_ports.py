# test_serial_ports.py
import unittest
import serial
import os

class TestSerialPorts(unittest.TestCase):
  def test_ttyS0_access(self):
    """Test if /dev/ttyS0 can be opened"""
    port = '/dev/ttyS0'
    try:
      ser = serial.Serial(port)
      self.assertTrue(ser.is_open)
      ser.close()
    except serial.SerialException as e:
      self.fail(f"Failed to open {port}: {str(e)}")

  def test_port_permissions(self):
    """Test if we have read/write permissions on ports"""
    ports = ['/dev/ttyS0']
    for port in ports:
      if os.path.exists(port):
        self.assertTrue(os.access(port, os.R_OK), f"No read permission on {port}")
        self.assertTrue(os.access(port, os.W_OK), f"No write permission on {port}")

if __name__ == '__main__':
  unittest.main()