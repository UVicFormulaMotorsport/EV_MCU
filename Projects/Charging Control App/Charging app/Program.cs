using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.IO.Ports;
using System.Management;

namespace Charging_app
{
    internal class Program
    {
        static void Main(string[] args)
        {
            char ch = 'y';
            while (ch == 'y')
            {
                list_devices();
                ch = Convert.ToChar(Console.Read());
            }
        }

        static void list_devices()
        {
            var usbDevices = GetUSBDevices();

            foreach (var usbDevice in usbDevices)
            {
                Console.WriteLine(
                    $"Device ID: {usbDevice.DeviceID}, PNP Device ID: {usbDevice.PnpDeviceID}, Description: {usbDevice.Description}");
            }

            string[] ports = SerialPort.GetPortNames();
            Console.WriteLine("The following serial ports were found:");

            foreach (var port in ports)
            {
                Console.WriteLine(port.ToString());
            }
        }


        static List<USBDeviceInfo> GetUSBDevices()
        {
            List<USBDeviceInfo> devices = new List<USBDeviceInfo>();

            using (var searcher = new ManagementObjectSearcher(
                @"Select * From Win32_USBHub"))
            {

                using (ManagementObjectCollection collection = searcher.Get())
                {

                    foreach (var device in collection)
                    {
                        devices.Add(new USBDeviceInfo(
                            (string)device.GetPropertyValue("DeviceID"),
                            (string)device.GetPropertyValue("PNPDeviceID"),
                            (string)device.GetPropertyValue("Description")
                            ));
                    }
                }
            }
            return devices;

        }
    }
}
