using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Microsoft.Kinect;
using Microsoft.Kinect.Toolkit;
using System.IO;

namespace KinectLowLevel
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {

        private readonly KinectSensorChooser _sensorChooser;
        private bool dumpNextFrame = false;
        private int index = 0;


        public MainWindow()
        {
            InitializeComponent();
            _sensorChooser = new KinectSensorChooser();
            _sensorChooser.Start();
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            // acquire sensor with sensorchooser
            _sensorChooser.KinectChanged += new EventHandler<KinectChangedEventArgs>(_sensorChooser_KinectChanged);

            KinectSensor newSensor = _sensorChooser.Kinect;

            newSensor.ColorStream.Enable(ColorImageFormat.RgbResolution1280x960Fps12);
            newSensor.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);
            newSensor.DepthStream.Range = DepthRange.Default;
            newSensor.SkeletonStream.Enable();
            newSensor.AllFramesReady +=
                new EventHandler<AllFramesReadyEventArgs>(_sensor_AllFramesReady);
            try
            {
                newSensor.Start();
            }
            catch (System.IO.IOException)
            {
                // some other thing is locking on this Kinect, try to fix it?
                _sensorChooser.TryResolveConflict();
            }
        }

        void _sensorChooser_KinectChanged(object sender, KinectChangedEventArgs e)
        {
            KinectSensor oldSensor = e.OldSensor;
            StopKinect(oldSensor);

            KinectSensor newSensor = e.NewSensor;

            if (newSensor == null)
            {
                return;
            }

            newSensor.ColorStream.Enable();
            newSensor.DepthStream.Enable();
            newSensor.SkeletonStream.Enable();
            newSensor.AllFramesReady +=
                new EventHandler<AllFramesReadyEventArgs>(_sensor_AllFramesReady);
            try
            {
                newSensor.Start();
            }
            catch (System.IO.IOException)
            {
                // some other thing is locking on this Kinect, try to fix it?
                _sensorChooser.TryResolveConflict();
            }
        }

        void StopKinect(KinectSensor sensor)
        {
            if (sensor != null)
            {
                if (sensor.IsRunning)
                {
                    sensor.Stop();
                    if (sensor.AudioSource != null)
                    {
                        sensor.AudioSource.Stop();
                    }
                }
            }
        }

        void _sensor_AllFramesReady(object sender, AllFramesReadyEventArgs e)
        {
            using (ColorImageFrame colorFrame = e.OpenColorImageFrame())
            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
            {
                if ((depthFrame == null) || (colorFrame == null))
                {
                    return; // frame dropped
                }

                byte[] pixels = new byte[colorFrame.PixelDataLength];
                colorFrame.CopyPixelDataTo(pixels);

                // from knowing that the image is BGR32
                int stride = colorFrame.Width * 4;

                // !!! TODO replace this with some kind modifiable thingy
                // !!! TODO the viewing can be done with a KinectColorViewer WPF thing
                colorImage.Source =
                    BitmapSource.Create(colorFrame.Width, colorFrame.Height,
                        96, 96, PixelFormats.Bgr32, null, pixels, stride);
                
                // depth processing
                DepthImagePixel[] dImPix = new DepthImagePixel[depthFrame.PixelDataLength];
                depthFrame.CopyDepthImagePixelDataTo(dImPix);

                ColorImagePoint[] cImPts = new ColorImagePoint[depthFrame.Width * depthFrame.Height];

                int dStride = depthFrame.Width * 4;

                _sensorChooser.Kinect.CoordinateMapper.MapDepthFrameToColorFrame(depthFrame.Format, dImPix,
                    colorFrame.Format, cImPts);
                // need to take the pixels from the depth frame and plot them at the coordinates of the other frame

                short[] depthShifted = new short[depthFrame.PixelDataLength];
                Array.Clear(depthShifted, 0, depthShifted.Length);

                int colorToDepthDivisor = colorFrame.Width / depthFrame.Width;
                // probably incredibly slow
                for (int i = 0; i < cImPts.Length; i++)
                {
                    int x = cImPts[i].X/colorToDepthDivisor;
                    int y = cImPts[i].Y/colorToDepthDivisor;
                    if ((x >= depthFrame.Width) || (y >= depthFrame.Height))
                    {
                        continue;
                    }

                    // we index from the upper left like civilized people
                    depthShifted[y * depthFrame.Width + x] =
                        dImPix[i].Depth;
                }

                byte[] depthIntensities = GenerateColoredBytes(depthShifted, depthFrame.Height, depthFrame.Width);

                // let's just slam the depth image over the other one right now
                //

                //byte[] dPixels = GenerateColoredBytes(depthFrame);

                depthImage.Source =
                    BitmapSource.Create(depthFrame.Width, depthFrame.Height,
                    96, 96, PixelFormats.Bgr32, null, depthIntensities, dStride);

                // debugging-level, change to make nicer
                if (dumpNextFrame == true)
                {
                    string filedir = @"C:\temp\kinectTest\";

                    #region color image dump
                    string bmpPath =
                        System.IO.Path.Combine(filedir, "c" + index.ToString() + ".bmp");

                    FileStream streamColor =
                        new FileStream(bmpPath, FileMode.Create);

                    BmpBitmapEncoder encoderColor = new BmpBitmapEncoder();
                    BitmapSource bitmapColor = BitmapSource.Create(colorFrame.Width,
                        colorFrame.Height, 96, 96, PixelFormats.Bgr32, null,
                        pixels, stride);
                    encoderColor.Frames.Add(BitmapFrame.Create(bitmapColor));
                    encoderColor.Save(streamColor);
                    #endregion

                    #region depth dump

                    // int depth = depthPoint >> DepthImageFrame.PlayerIndexBitmaskWidth
                    //using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
                    
                    if (depthFrame == null)
                    {
                        index++;
                        dumpNextFrame = false;
                        return;
                    }

                    short[] dRaw = GenerateDepthShort(depthFrame);

                    // I know we don't have to escape anything so :effort:
                    string depthString = string.Join(",", dRaw.Select<short, string>((x) => x.ToString()));

                    string depthPath =
                    System.IO.Path.Combine(filedir, "d" + index.ToString() + ".csv");

                    //FileStream streamDepth =
                    //    new FileStream(depthPath, FileMode.Create);

                    File.WriteAllText(depthPath, depthString);

                    #endregion


                    #region image correspondance dump

                    // LINQ yo
                    string xMapStr = string.Join(",", cImPts.Select(x => x.X));
                    string xMapPath = System.IO.Path.Combine(filedir, "x" + index.ToString() + ".csv");

                    File.WriteAllText(xMapPath, xMapStr);

                    string yMapStr = string.Join(",", cImPts.Select(x => x.Y));
                    string yMapPath = System.IO.Path.Combine(filedir, "y" + index.ToString() + ".csv");

                    File.WriteAllText(yMapPath, yMapStr);

                    #endregion


                    dumpNextFrame = false;
                    index++;
                    
                }
            }
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            StopKinect(_sensorChooser.Kinect);
        }

        private void button1_Click(object sender, RoutedEventArgs e)
        {
            dumpNextFrame = true;
        }

        private short[] GenerateDepthShort(DepthImageFrame depthFrame)
        {
            short[] rawDepthData = new short[depthFrame.PixelDataLength];
            depthFrame.CopyPixelDataTo(rawDepthData);

            // holy moley all of the depths were off by like 8 before, hehhheh

            for (int i = 0; i < rawDepthData.Length; i++)
            {
                rawDepthData[i] = (short)(rawDepthData[i] >> DepthImageFrame.PlayerIndexBitmaskWidth);
            }
            return rawDepthData;
        }

        private byte[] GenerateColoredBytes(short[] depths, int frameHeight, int frameWidth)
        {
            byte[] pixels = new byte[frameHeight * frameWidth * 4];

            const int BlueIndex = 0;
            const int GreenIndex = 1;
            const int RedIndex = 2;

            for (int depthIndex = 0, colorIndex = 0;
                depthIndex < depths.Length && colorIndex < pixels.Length;
                depthIndex++, colorIndex += 4)
            {
                // 13 top bits are the useful part, but we don't have to pack down because it's already there
                //int player = depths[depthIndex] & DepthImageFrame.PlayerIndexBitmask;

                //int depth = depths[depthIndex] >> DepthImageFrame.PlayerIndexBitmaskWidth;
                int depth = depths[depthIndex];
                byte intensity = CalculateIntensityFromDepth(depth);

                pixels[colorIndex + BlueIndex] = intensity;
                pixels[colorIndex + GreenIndex] = intensity;
                pixels[colorIndex + RedIndex] = intensity;
            }

            return pixels;
        }

        private byte[] GenerateColoredBytes(DepthImageFrame depthFrame)
        {
            short[] rawDepthData = new short[depthFrame.PixelDataLength];
            depthFrame.CopyPixelDataTo(rawDepthData);

            byte[] pixels = new byte[depthFrame.Height * depthFrame.Width * 4];

            const int BlueIndex = 0;
            const int GreenIndex = 1;
            const int RedIndex = 2;

            for (int depthIndex = 0, colorIndex = 0;
                depthIndex < rawDepthData.Length && colorIndex < pixels.Length;
                depthIndex++, colorIndex += 4)
            {
                int player = rawDepthData[depthIndex] & DepthImageFrame.PlayerIndexBitmask;

                int depth = rawDepthData[depthIndex] >> DepthImageFrame.PlayerIndexBitmaskWidth;

                byte intensity = CalculateIntensityFromDepth(depth);

                pixels[colorIndex + BlueIndex] = intensity;
                pixels[colorIndex + GreenIndex] = intensity;
                pixels[colorIndex + RedIndex] = intensity;
            }

            return pixels;
        }

        // from 800mm to 4000mm according to the thing, rescale onto intensity axes
        public static byte CalculateIntensityFromDepth(int distance)
        {
            return (byte)(255 - (255 * Math.Max(distance - 800, 0) / 4000));
        }
    }
}
