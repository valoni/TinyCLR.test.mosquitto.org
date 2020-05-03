using System;
using System.Text;
using System.Diagnostics;
using System.Threading;

using System.Net;
using System.Net.Sockets;
using GHIElectronics.TinyCLR.Devices.Network;
using GHIElectronics.TinyCLR.Networking.Mqtt;

using GHIElectronics.TinyCLR.Devices.Gpio;

using GHIElectronics.TinyCLR.Devices.Spi;
using GHIElectronics.TinyCLR.Pins;

using System.Security.Cryptography.X509Certificates;


using GHIElectronics.TinyCLR.Native;



namespace test.mosquitto.org
{
    class Program
    {
        private static Mqtt client;

        private static bool linkReady = false;
        private static NetworkController networkController;

        private int cntrs = 0;

        static void Main()
        {

            Debug.WriteLine("Setup Ethernet");
            SetupEnc28_MicroBus(SC20100.SpiBus.Spi3,SC20100.GpioPin.PD14,SC20100.GpioPin.PA8,SC20100.GpioPin.PD15,SC20100.GpioPin.Id);
            Thread.Sleep(50);

            Debug.WriteLine("Setup Date and Time from Internet");
            Debug.WriteLine("===================================");

            while
            ((DateTime.UtcNow.ToString("yyyy.MM.dd")=="1900.01.01")|| (DateTime.UtcNow.ToString("yyyy.MM.dd") == "2017.01.01"))
            { 
              SystemTime.SetTime(GetNetworkTime(2));
                Thread.Sleep(15000);
               // Debug.WriteLine("Try again after 15 sec");
            }
          
            Debug.WriteLine("Set Proper Time now is :" + DateTime.UtcNow.ToString("yyyy.MM.dd HH:mm:ss"));
            Debug.WriteLine("===================================");

            //test.mosquitto.org
            var caCertificate = new X509Certificate(Certificate());

            var mqttHost = "test.mosquitto.org";
            //var mqttHost = "broker.hivemq.com";

            var mqttPort = 1883;//1883 no ssl - 8883 with ssl
            var deviceId = "tiny_clr_device_001";

            string username = null;
            string password = null;

            string topic1 = "tinyclr/temp";
            string topic2 = "tinyclr/humidity";

            string subscribe1 = "tinyclr/light1";
            string subscribe2 = "tinyclr/light2";

            var clientSetting = new MqttClientSetting
                {
                    BrokerName = mqttHost,
                    BrokerPort = mqttPort,
                  //  ClientCertificate = null,
                  //  CaCertificate = caCertificate,
                  //  SslProtocol = System.Security.Authentication.SslProtocols.Tls12
                };

            try
            {
                client = new Mqtt(clientSetting);
            }
            catch (Exception e)
            {
                Debug.WriteLine("Failure one");
                Debug.WriteLine(e.Message.ToString());
            }

        
            var connectSetting = new MqttConnectionSetting
           {
               ClientId = deviceId,
               UserName = username,
               Password = password,
               KeepAliveTimeout = 60,
               LastWillQos = QoSLevel.MostOnce
                /*
                  QOS(0) = MostOnce 
                  QOS(1) = LeastOnce
                  QOS(2) = ExactlyOnce
                */
            };

            //Connect to host

        
            var returnCode = client.Connect(connectSetting);
            int packetId = 1;

            client.PublishReceivedChanged += Client_PublishReceivedChanged;

            client.SubscribedChanged += Client_SubscribedChanged;



            // Subscribe to a topic
             client.Subscribe(new string[] { subscribe1, subscribe2 }, new QoSLevel[] { QoSLevel.MostOnce, QoSLevel.MostOnce }, (ushort)packetId++); // there was no QoS Level for the second topic in your code
   

            while (true)
                {
                Debug.WriteLine("------------------------------");
                Debug.WriteLine(DateTime.UtcNow.ToString(">> yyyy.MM.dd HH:mm:ss.fff"));

                //Publish a topic
                client.Publish(topic1, GetRandomTemperature(), QoSLevel.MostOnce, false, (ushort)packetId);
                packetId++;


             

                //Publish a topic
                client.Publish(topic2, GetRandomHumidity(), QoSLevel.MostOnce, false, (ushort)packetId);
                packetId++;


                client.PublishedChanged += (a, b, c) =>
                {
                    Debug.WriteLine("Published " + a.ToString());
                };

                Debug.WriteLine("Wait next 15 sec");
                Thread.Sleep(15000);
            }

        }

        private static void Client_SubscribedChanged(object sender, MqttPacket packet)
        { 
            Debug.WriteLine("subscribe " + DateTime.UtcNow.ToString("yyyy.MM.dd HH:mm:ss.fff"));
        }

        //should this work under subscribe changed .....
        private static void Client_PublishReceivedChanged(object sender, MqttPacket packet)
        {
            string rez= UTF8Encoding.UTF8.GetString(packet.Data);
          //  Debug.WriteLine("Sender : " + sender.ToString());
          //  Debug.WriteLine("Message : " + packet.ToString());
             switch(rez)
             {
                case "0":Debug.WriteLine("Light / Off");
                    break;
                case "1":Debug.WriteLine("Light / On");
                    break;
                case "10":Debug.WriteLine("Switch / Off");
                    break;
                case "11":Debug.WriteLine("Switch / On");
                    break;
             }
        }

        //ETHERNET SETUP
        static void SetupEnc28_MicroBus(string _SpiApiName,int _cs,int _intPin,int _rst, string _GpioApiName)
        {
            networkController = NetworkController.FromName("GHIElectronics.TinyCLR.NativeApis.ENC28J60.NetworkController");

            var networkInterfaceSetting = new EthernetNetworkInterfaceSettings();
            var networkCommunicationInterfaceSettings = new SpiNetworkCommunicationInterfaceSettings();

            networkCommunicationInterfaceSettings.SpiApiName = _SpiApiName;
            networkCommunicationInterfaceSettings.GpioApiName = _GpioApiName;

            var cs = GpioController.GetDefault().OpenPin(_cs);

            var settings = new SpiConnectionSettings()
            {
                ChipSelectLine = cs,
                ClockFrequency = 4000000,
                Mode = SpiMode.Mode0,
                ChipSelectType = SpiChipSelectType.Gpio,
                ChipSelectHoldTime = TimeSpan.FromTicks(10),
                ChipSelectSetupTime = TimeSpan.FromTicks(10)
            };

            networkCommunicationInterfaceSettings.SpiSettings = settings;

            networkCommunicationInterfaceSettings.InterruptPin = GpioController.GetDefault().OpenPin(_intPin);
            networkCommunicationInterfaceSettings.InterruptEdge = GpioPinEdge.FallingEdge;
            networkCommunicationInterfaceSettings.InterruptDriveMode = GpioPinDriveMode.InputPullUp;

            networkCommunicationInterfaceSettings.ResetPin = GpioController.GetDefault().OpenPin(_rst);
            networkCommunicationInterfaceSettings.ResetActiveState = GpioPinValue.Low;

            networkInterfaceSetting.Address = new IPAddress(new byte[] { 192, 168, 2, 122 });
            networkInterfaceSetting.SubnetMask = new IPAddress(new byte[] { 255, 255, 255, 0 });
            networkInterfaceSetting.GatewayAddress = new IPAddress(new byte[] { 192, 168, 2, 1 });
            networkInterfaceSetting.DnsAddresses = new IPAddress[] { new IPAddress(new byte[] { 8, 8, 8, 8 }), new IPAddress(new byte[] { 75, 75, 75, 76 }) };

            networkInterfaceSetting.MacAddress = new byte[] { 0x00, 0x4, 0x00, 0x00, 0x00, 0x00 };
            networkInterfaceSetting.IsDhcpEnabled = true;
            networkInterfaceSetting.IsDynamicDnsEnabled = true;

            networkInterfaceSetting.TlsEntropy = new byte[] { 0, 1, 2, 3 };

            networkController.SetInterfaceSettings(networkInterfaceSetting);
            networkController.SetCommunicationInterfaceSettings(networkCommunicationInterfaceSettings);

            networkController.SetAsDefaultController();

            networkController.NetworkAddressChanged += NetworkController_NetworkAddressChanged; ;
            networkController.NetworkLinkConnectedChanged += NetworkController_NetworkLinkConnectedChanged; ;

            networkController.Enable();

            while (linkReady == false) ;
            Debug.WriteLine("===================================");
            Debug.WriteLine("Network is ready to use:");
            Debug.WriteLine("===================================");
            Debug.WriteLine(networkController.GetIPProperties().Address.ToString());
            Debug.WriteLine(networkController.GetIPProperties().SubnetMask.ToString());
            Debug.WriteLine(networkController.GetIPProperties().GatewayAddress.ToString());
            Debug.WriteLine(networkController.GetIPProperties().DnsAddresses[1].ToString());
            Debug.WriteLine(UTF8Encoding.UTF8.GetString(networkController.GetInterfaceProperties().MacAddress));
            Debug.WriteLine("===================================");

        }

        private static void NetworkController_NetworkLinkConnectedChanged(NetworkController sender, NetworkLinkConnectedChangedEventArgs e)
        {
            //throw new NotImplementedException();
        }

        private static void NetworkController_NetworkAddressChanged(NetworkController sender, NetworkAddressChangedEventArgs e)
        {
            var ipProperties = sender.GetIPProperties();
            var address = ipProperties.Address.GetAddressBytes();

            linkReady = address[0] != 0;
        }

        public static DateTime GetNetworkTime(int CorrectLocalTime = 0)
        {
            const string ntpServer = "pool.ntp.org";
            var ntpData = new byte[48];
            ntpData[0] = 0x1B; //LeapIndicator = 0 (no warning), VersionNum = 3 (IPv4 only), Mode = 3 (Client Mode)

            var addresses = Dns.GetHostEntry(ntpServer).AddressList;
            var ipEndPoint = new IPEndPoint(addresses[0], 123);
            var socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);

            socket.Connect(ipEndPoint);

            Thread.Sleep(10); // added to support TinyCLR OS too

            //addedd time out for TinyCLR OS ..
            socket.ReceiveTimeout = 5000;
            socket.SendTimeout = 5000;

            socket.Send(ntpData);

            socket.Receive(ntpData);

            socket.Close();

            ulong intPart = (ulong)ntpData[40] << 24 | (ulong)ntpData[41] << 16 | (ulong)ntpData[42] << 8 | (ulong)ntpData[43];
            ulong fractPart = (ulong)ntpData[44] << 24 | (ulong)ntpData[45] << 16 | (ulong)ntpData[46] << 8 | (ulong)ntpData[47];

            var milliseconds = (intPart * 1000) + ((fractPart * 1000) / 0x100000000L);
            var networkDateTime = (new DateTime(1900, 1, 1)).AddMilliseconds((long)milliseconds);

            return networkDateTime.AddHours(CorrectLocalTime);
        }

        //simulate random Temperature ..
        private static byte[] GetRandomTemperature()
        {
            // generate random value
            Random randomProvider = new Random();
            var randomTemperature = randomProvider.NextDouble() * 10;

            // convert to string formatted NN.NN
            var temperatureAsString = randomTemperature.ToString("N2");

            Debug.WriteLine($"Temperature: {temperatureAsString}");

            return Encoding.UTF8.GetBytes(temperatureAsString);
        }

        private static byte[] GetRandomHumidity()
        {
            // generate random value
            Random randomProvider = new Random();
            var randomTemperature = randomProvider.NextDouble() * 100;

            // convert to string formatted NN.NN
            var temperatureAsString = randomTemperature.ToString("N2");

            Debug.WriteLine($"Humidity: {temperatureAsString}");

            return Encoding.UTF8.GetBytes(temperatureAsString);
        }

        static byte[] Certificate()
        { 
        // Mosquito test server CA certificate
        // from http://test.mosquitto.org/

        // X509 
        string GetCertificate =
@"-----BEGIN CERTIFICATE-----
MIIC8DCCAlmgAwIBAgIJAOD63PlXjJi8MA0GCSqGSIb3DQEBBQUAMIGQMQswCQYD
VQQGEwJHQjEXMBUGA1UECAwOVW5pdGVkIEtpbmdkb20xDjAMBgNVBAcMBURlcmJ5
MRIwEAYDVQQKDAlNb3NxdWl0dG8xCzAJBgNVBAsMAkNBMRYwFAYDVQQDDA1tb3Nx
dWl0dG8ub3JnMR8wHQYJKoZIhvcNAQkBFhByb2dlckBhdGNob28ub3JnMB4XDTEy
MDYyOTIyMTE1OVoXDTIyMDYyNzIyMTE1OVowgZAxCzAJBgNVBAYTAkdCMRcwFQYD
VQQIDA5Vbml0ZWQgS2luZ2RvbTEOMAwGA1UEBwwFRGVyYnkxEjAQBgNVBAoMCU1v
c3F1aXR0bzELMAkGA1UECwwCQ0ExFjAUBgNVBAMMDW1vc3F1aXR0by5vcmcxHzAd
BgkqhkiG9w0BCQEWEHJvZ2VyQGF0Y2hvby5vcmcwgZ8wDQYJKoZIhvcNAQEBBQAD
gY0AMIGJAoGBAMYkLmX7SqOT/jJCZoQ1NWdCrr/pq47m3xxyXcI+FLEmwbE3R9vM
rE6sRbP2S89pfrCt7iuITXPKycpUcIU0mtcT1OqxGBV2lb6RaOT2gC5pxyGaFJ+h
A+GIbdYKO3JprPxSBoRponZJvDGEZuM3N7p3S/lRoi7G5wG5mvUmaE5RAgMBAAGj
UDBOMB0GA1UdDgQWBBTad2QneVztIPQzRRGj6ZHKqJTv5jAfBgNVHSMEGDAWgBTa
d2QneVztIPQzRRGj6ZHKqJTv5jAMBgNVHRMEBTADAQH/MA0GCSqGSIb3DQEBBQUA
A4GBAAqw1rK4NlRUCUBLhEFUQasjP7xfFqlVbE2cRy0Rs4o3KS0JwzQVBwG85xge
REyPOFdGdhBY2P1FNRy0MDr6xr+D2ZOwxs63dG1nnAnWZg7qwoLgpZ4fESPD3PkA
1ZgKJc2zbSQ9fCPxt2W3mdVav66c6fsb7els2W2Iz7gERJSX
-----END CERTIFICATE-----";

            return UTF8Encoding.UTF8.GetBytes(GetCertificate);
    }
}
}
