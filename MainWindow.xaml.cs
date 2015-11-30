

namespace Microsoft.Samples.Kinect.SkeletonBasics
{
    using System;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Controls;
    using System.Windows.Data;
    using System.Windows.Input;
    using System.Windows.Forms;
    using System.Windows.Media.Imaging;
    using System.Runtime.Serialization.Formatters.Binary;

    using System.ComponentModel;
    using System.Runtime.InteropServices;

    using System.Collections; // Para la cola

    using Microsoft.Kinect;
    using Microsoft.Kinect.Toolkit;
    using Microsoft.Kinect.Toolkit.Interaction;
    // Tiene conflicto con otro using en la definición de Point, así se puede evitar de forma cómoda
    using systemdrawing = System.Drawing;

    // Cosas copy&paste, para que se tenga una clase propia heredada de IInteractionClient que posteriormente se utiliza para el InteractionStream
    public class DummyInteractionClient : IInteractionClient
    {
        public InteractionInfo GetInteractionInfoAtLocation(
            int skeletonTrackingId,
            InteractionHandType handType,
            double x,
            double y)
        {
            var result = new InteractionInfo();
            result.IsGripTarget = true;
            result.IsPressTarget = true;
            result.PressAttractionPointX = 0.5;
            result.PressAttractionPointY = 0.5;
            result.PressTargetControlId = 1;

            return result;
        }
    }
    


    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        /// <summary>
        /// Width of output drawing
        /// </summary>
        private const float RenderWidth = 640.0f;

        /// <summary>
        /// Height of our output drawing
        /// </summary>
        private const float RenderHeight = 480.0f;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of body center ellipse
        /// </summary>
        private const double BodyCenterThickness = 10;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Brush used to draw skeleton center point
        /// </summary>
        private readonly Brush centerPointBrush = Brushes.Blue;

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently tracked
        /// </summary>
        private readonly Pen trackedBonePen = new Pen(Brushes.Green, 6);

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor sensor;

        /// <summary>
        /// Drawing group for skeleton rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;

        /// <summary>
        /// Bitmap that will hold color information
        /// </summary>
        private WriteableBitmap colorBitmap;

        /// <summary>
        /// Intermediate storage for the color data received from the camera
        /// </summary>
        private byte[] colorPixels;

        private InteractionStream interactionStream;
        private UserInfo[] userInfos;

        // ------------------------------------------------------------------
        // PROPIO
        // ------------------------------------------------------------------

        // Altura y anchura del sistema
        int altura = System.Windows.Forms.SystemInformation.PrimaryMonitorSize.Height;
        int anchura = System.Windows.Forms.SystemInformation.PrimaryMonitorSize.Width;

        private const UInt32 MOUSEEVENTF_LEFTDOWN = 0x0002;     // Constantes
        private const UInt32 MOUSEEVENTF_LEFTUP = 0x0004;       // para los
        private const UInt32 MOUSEEVENTF_RIGHTDOWN = 0x0008;    // eventos
        private const UInt32 MOUSEEVENTF_RIGHTUP = 0x0010;      // del ratón

        [DllImport("user32.dll")] // Importa la DLL del sistema
        private static extern void mouse_event(uint dwFlags, uint dx, uint dy, uint dwData, uint dwExtraInf); // Específicamente esta función


        const double x1 = 0;    // Coordenada X de la esquina superior izquierda de la mano
        const double y1 = 0.3;  // Coordenada Y de la esquina superior izquierda de la mano
        const double x2 = 0.4;  // Coordenada X de la esquina inferior derecha de la mano
        const double y2 = 0.1;  // Coordenada Y de la esquina inferior derecha de la mano


        Queue anteriores; // Cola que mantiene los últimos 15 puntos leídos de la mano
        systemdrawing.Point media; // Media de dicha cola (ver la función suavizado para más explicaciones)
        const int TAM_COLA = 15; // Tamaño de la cola (a mayor, movimientos más suavizados y retardados)


        // Control de modos
        int modo;                 // Modo actual (1, 2, 3, 4)
        bool bloquear = false;    // Si TRUE, está en modo bloqueo (modo 2) dentro del modo normal. El modo bloqueo solo es temporal dentro del modo normal.
        bool sobrecabeza = false; // Si TRUE, la mano está sobre la cabeza (no cambia de modo hasta que pasa a estar falso).
        bool encendido = true;    // Si TRUE, está en los modos normal (1), bloqueo (2) o diapositiva (4).
        bool diapositiva = false; // Si TRUE, está en el modo diapositiva (4).

        // [modo 1 y 2] ZONA (1, 2, 3, 4, 5) respecto del pecho del último frame (-1 es por defecto y no hace nada).
        // Utilidad: en caso de que el frame actual esté en zona 1 (detrás del pecho), hace un click derecho y pone anterior a 1.
        // En el siguiente frame, si sigue estando detrás del pecho, como anterior está en zona 1, no envía de nuevo un click derecho
        // Si no estuviera esta var, si se deja la mano 30 frames detrás del pecho, mandaría 30 clicks (en solo un segundo), en vez de solo 1
        int anterior = -1;

        // Control del puño cerrado (funciona en modos: 1 normal, 2 bloqueo y 4 diapositiva)
        bool manoderechacerrada = false;   // Si TRUE, la máquina detectó el puño cerrado pero no ha visto que se haya abierto
        bool manoizquierdacerrada = false;
        int contador_manoderechacerrada = 0; // Cuenta el número de FRAMES con manoderechacerrada == true
        int contador_manoizquierdacerrada = 0;

        // Tiempo (en frames, cada 30 fr -> 1 seg) que:
        // - En modos normal (1) y bloqueo (2), va a provocar que no haga un click normal, sino que se deje presionado
        // - En modo diapositiva, cambie de diapositiva
        const int TIEMPO_CERRADO = 50;




        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            InitializeComponent();
        }


        // Esta función se llama cada vez que un FRAME del INTERACTION STREAM está listo
        private void InteractionStreamOnInteractionFrameReady(object sender, InteractionFrameReadyEventArgs e)
        {
            using (InteractionFrame frame = e.OpenInteractionFrame())
            { // Cosas que hacen falta para que funcione
                if (frame != null)
                {
                    if (this.userInfos == null)
                    {
                        this.userInfos = new UserInfo[InteractionFrame.UserInfoArrayLength];
                    }

                    frame.CopyInteractionDataTo(this.userInfos);
                }
                else
                {
                    return;
                }
            }

            foreach (UserInfo userInfo in this.userInfos)
            { // Por cada usuario que esté siguiéndose
                foreach (InteractionHandPointer handPointer in userInfo.HandPointers)
                { // Por cada "mano" que tenga dicho usuario
                    if (handPointer.HandType == InteractionHandType.Left)
                    { // Mano izquierda
                        if (handPointer.HandEventType == InteractionHandEventType.Grip)
                        { // Cierra el puño
                            manoizquierdacerrada = true;
                        }
                        else if (handPointer.HandEventType == InteractionHandEventType.GripRelease)
                        { // Abre la mano
                            manoizquierdacerrada = false;
                        }
                    }
                    else if (handPointer.HandType == InteractionHandType.Right)
                    { // Mano derecha
                        if (handPointer.HandEventType == InteractionHandEventType.Grip)
                        { // Cierra el puño
                            manoderechacerrada = true;
                        }
                        else if (handPointer.HandEventType == InteractionHandEventType.GripRelease)
                        { // Abre la mano
                            manoderechacerrada = false;
                        }
                    }

                }
            }
        }



        /// <summary>
        /// Draws indicators to show which edges are clipping skeleton data
        /// </summary>
        /// <param name="skeleton">skeleton to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private static void RenderClippedEdges(Skeleton skeleton, DrawingContext drawingContext)
        {
            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, RenderHeight - ClipBoundsThickness, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, RenderHeight));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(RenderWidth - ClipBoundsThickness, 0, ClipBoundsThickness, RenderHeight));
            }
        }

        /// <summary>
        /// Execute startup tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
            // Llama al constructor para la cola
            anteriores = new Queue();

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // Display the drawing using our image control
            Image.Source = this.imageSource;

            // Look through all sensors and start the first connected one.
            // This requires that a Kinect is connected at the time of app startup.
            // To make your app robust against plug/unplug, 
            // it is recommended to use KinectSensorChooser provided in Microsoft.Kinect.Toolkit (See components in Toolkit Browser).
            foreach (var potentialSensor in KinectSensor.KinectSensors)
            {
                if (potentialSensor.Status == KinectStatus.Connected)
                {
                    this.sensor = potentialSensor;
                    break;
                }
            }

            if (null != this.sensor)
            {
                this.sensor.DepthStream.Range = DepthRange.Default;
                this.sensor.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);

                // Turn on the skeleton stream to receive skeleton frames
                this.sensor.SkeletonStream.Enable();

                // Add an event handler to be called whenever there is new color frame data
                this.sensor.SkeletonFrameReady += this.SensorSkeletonFrameReady;

                // Add an event handler to be called whenever there is new color frame data
                this.sensor.ColorFrameReady += this.SensorColorFrameReady;

                // Turn on the color stream to receive color frames
                this.sensor.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);

                // Allocate space to put the pixels we'll receive
                this.colorPixels = new byte[this.sensor.ColorStream.FramePixelDataLength];

                // This is the bitmap we'll display on-screen
                this.colorBitmap = new WriteableBitmap(this.sensor.ColorStream.FrameWidth, this.sensor.ColorStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null);

                this.interactionStream = new InteractionStream(this.sensor,  new DummyInteractionClient());
                this.interactionStream.InteractionFrameReady += InteractionStreamOnInteractionFrameReady;

                this.sensor.DepthFrameReady += SensorDepthFrameReady;

                // Start the sensor!
                try
                {
                    this.sensor.Start();
                }
                catch (IOException)
                {
                    this.sensor = null;
                }
            }

            if (null == this.sensor)
            {
                this.statusBarText.Text = Properties.Resources.NoKinectReady;
            }

            try
            { // Obtiene el ángulo de la cámara, para que la palanca lo tenga
                alturaslider.Value = sensor.ElevationAngle;
            }
            catch { }
        }



        /// <summary>
        /// Event handler for Kinect sensor's ColorFrameReady event
        /// EN CADA COLORFRAME SE COMUNICA CON LA PANTALLA PARA QUE SE VEAN
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void SensorColorFrameReady(object sender, ColorImageFrameReadyEventArgs e)
        {
            using (ColorImageFrame colorFrame = e.OpenColorImageFrame())
            {
                if (colorFrame != null)
                {
                    // Copy the pixel data from the image to a temporary array
                    colorFrame.CopyPixelDataTo(this.colorPixels);

                    // Write the pixel data into our bitmap
                    this.colorBitmap.WritePixels(
                        new Int32Rect(0, 0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight),
                        this.colorPixels,
                        this.colorBitmap.PixelWidth * sizeof(int),
                        0);
                }
            }
        }


        // INDISPENSABLE PARA CIERRE DEL PUÑO, PASA INFORMACIÓN DE LA PROFUNDIDAD EN CADA FRAME AL INTERACTIONSTREAM
        private void SensorDepthFrameReady(object sender, DepthImageFrameReadyEventArgs depthImageFrameReadyEventArgs)
        {
            using (DepthImageFrame depthFrame = depthImageFrameReadyEventArgs.OpenDepthImageFrame())
            {
                if (depthFrame == null)
                    return;
                try
                {
                    interactionStream.ProcessDepth(depthFrame.GetRawPixelData(), depthFrame.Timestamp);
                }
                catch (InvalidOperationException)
                {
                // DepthFrame functions may throw when the sensor gets
                // into a bad state.  Ignore the frame in that case.
                }
            }
        }


        /// <summary>
        /// Execute shutdown tasks
        /// CIERRA
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (null != this.sensor)
            {
                this.sensor.Stop();
            }
        }

        /// <summary>
        /// Event handler for Kinect sensor's SkeletonFrameReady event
        /// SE DISPARA CON CADA FRAME DEL SKELETON Y HACE LAS COSAS TÍPICAS DEL SKELETONBASICS
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void SensorSkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            Skeleton[] skeletons = new Skeleton[0];

            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletons);

                    try
                    {
                        var accelerometerReading = sensor.AccelerometerGetCurrentReading();
                        interactionStream.ProcessSkeleton(skeletons, accelerometerReading, skeletonFrame.Timestamp);
                    }
                    catch (InvalidOperationException)
                    {

                    }
                }
            }


            using (DrawingContext dc = this.drawingGroup.Open())
            {
                // Draw a transparent background to set the render size
                dc.DrawImage(this.colorBitmap, new Rect(0.0, 0.0, RenderWidth, RenderHeight));


                if (skeletons.Length != 0)
                {
                    foreach (Skeleton skel in skeletons)
                    {
                        RenderClippedEdges(skel, dc);

                        if (skel.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            this.DrawBonesAndJoints(skel, dc);
                            break; // Solo tiene en cuenta la primera persona, no analiza más
                        }
                    }
                }

                // prevent drawing outside of our render area
                this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, RenderWidth, RenderHeight));
            }
            if (skeletons.Length != 0)
                foreach (Skeleton skel in skeletons)
                {
                    if (skel.TrackingState == SkeletonTrackingState.Tracked)
                    {
                        // SI TODO HA IDO BIEN y hay al menos una persona, SE LLAMA A General(Skeleton)
                        General(skel);
                        break;
                    }
                }


        }

        /// <summary>
        /// Draws a skeleton's bones and joints
        /// PINTA HUESOS (LA FUNCIÓN DRAWBONE NO ES DEL SISTEMA O DE LA API, ESTÁ DEFINIDA MÁS ABAJO)
        /// </summary>
        /// <param name="skeleton">skeleton to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawBonesAndJoints(Skeleton skeleton, DrawingContext drawingContext)
        {
            // Render Torso
            this.DrawBone(skeleton, drawingContext, JointType.Head, JointType.ShoulderCenter);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderRight);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.Spine);
            this.DrawBone(skeleton, drawingContext, JointType.Spine, JointType.HipCenter);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipLeft);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipRight);

            // Left Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderLeft, JointType.ElbowLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowLeft, JointType.WristLeft);
            this.DrawBone(skeleton, drawingContext, JointType.WristLeft, JointType.HandLeft);

            // Right Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderRight, JointType.ElbowRight);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowRight, JointType.WristRight);
            this.DrawBone(skeleton, drawingContext, JointType.WristRight, JointType.HandRight);

            // Left Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipLeft, JointType.KneeLeft);
            this.DrawBone(skeleton, drawingContext, JointType.KneeLeft, JointType.AnkleLeft);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleLeft, JointType.FootLeft);

            // Right Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipRight, JointType.KneeRight);
            this.DrawBone(skeleton, drawingContext, JointType.KneeRight, JointType.AnkleRight);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleRight, JointType.FootRight);
 
            // Render Joints
            foreach (Joint joint in skeleton.Joints)
            {
                Brush drawBrush = null;

                if (joint.TrackingState == JointTrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;                    
                }
                else if (joint.TrackingState == JointTrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;                    
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, this.SkeletonPointToScreen(joint.Position), JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Maps a SkeletonPoint to lie within our render space and converts to Point
        /// TRANSFORMA UN PUNTO DEL SKELETONPOINT A UN PUNTO BIDIMENSIONAL DE LA PANTALLA
        /// </summary>
        /// <param name="skelpoint">point to map</param>
        /// <returns>mapped point</returns>
        private Point SkeletonPointToScreen(SkeletonPoint skelpoint)
        {
            // Convert point to depth space.  
            // We are not using depth directly, but we do want the points in our 640x480 output resolution.
            DepthImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skelpoint, DepthImageFormat.Resolution640x480Fps30);
            return new Point(depthPoint.X, depthPoint.Y);
        }

        /// <summary>
        /// Draws a bone line between two joints
        /// VIENE DE SKELETONBASICS, PINTA UNA LÍNEA QUE REPRESENTA UN HUESO
        /// </summary>
        /// <param name="skeleton">skeleton to draw bones from</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="jointType0">joint to start drawing from</param>
        /// <param name="jointType1">joint to end drawing at</param>
        private void DrawBone(Skeleton skeleton, DrawingContext drawingContext, JointType jointType0, JointType jointType1)
        {
            Joint joint0 = skeleton.Joints[jointType0];
            Joint joint1 = skeleton.Joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == JointTrackingState.NotTracked ||
                joint1.TrackingState == JointTrackingState.NotTracked)
            {
                return;
            }

            // Don't draw if both points are inferred
            if (joint0.TrackingState == JointTrackingState.Inferred &&
                joint1.TrackingState == JointTrackingState.Inferred)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if (joint0.TrackingState == JointTrackingState.Tracked && joint1.TrackingState == JointTrackingState.Tracked)
            {
                drawPen = this.trackedBonePen;
            }

            drawingContext.DrawLine(drawPen, this.SkeletonPointToScreen(joint0.Position), this.SkeletonPointToScreen(joint1.Position));
        }

        /// <summary>
        /// Handles the checking or unchecking of the seated mode combo box
        /// ESTO VIENE EN SKELETONBASICS, PERO EN NUESTRA CÁMARA NO SIRVE PARA NADA
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void CheckBoxSeatedModeChanged(object sender, RoutedEventArgs e)
        {
            if (null != this.sensor)
            {
                if (this.checkBoxSeatedMode.IsChecked.GetValueOrDefault())
                {
                    this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;
                }
                else
                {
                    this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Default;
                }
            }
        }

        // --------------------------------------------------------------------------------------------------------------
        // Propio
        // --------------------------------------------------------------------------------------------------------------

        private void Click_botonprimario(object sender, EventArgs e)
        { // Llama al evento de presionar el botón izquierdo y soltar el botón izquierdo
            mouse_event(MOUSEEVENTF_LEFTDOWN, 0, 0, 0, 0);  // DOWN de pulsar
            mouse_event(MOUSEEVENTF_LEFTUP, 0, 0, 0, 0);    // UP del dedo para arriba
        }
        private void Click_botonsecundario(object sender, EventArgs e)
        { // Llama al evento de presionar el botón derecho y soltar el botón derecho
            mouse_event(MOUSEEVENTF_RIGHTDOWN, 0, 0, 0, 0);
            mouse_event(MOUSEEVENTF_RIGHTUP, 0, 0, 0, 0);
        }
        private void Mantener_botonprimario(object sender, EventArgs e)
        {
            mouse_event(MOUSEEVENTF_LEFTDOWN, 0, 0, 0, 0);
        }
        private void Soltar_botonprimario(object sender, EventArgs e)
        {
            mouse_event(MOUSEEVENTF_LEFTUP, 0, 0, 0, 0);
        }
        private void Mover(int a, int b)
        { // Accede al CURSOR del SISTEMA y añade un objeto POINT de la librería SYSTEM.DRAWING con las nuevas coordenadas del monitor
            System.Windows.Forms.Cursor.Position = new systemdrawing::Point(a, b);
        }
        private void Mover(systemdrawing.Point p)
        { // Accede al CURSOR del SISTEMA y añade un objeto POINT de la librería SYSTEM.DRAWING con las nuevas coordenadas del monitor
            System.Windows.Forms.Cursor.Position = p;
        }


        // Función de suavizado: calcula el promedio de los últimos TAM_COLA puntos recibidos
        // Aclaración: media [x1, x2, x3, ..., xn] := (x1 + x2 + x3 + ... + xn) / n = x1 / n + x2 / n + x3 / n + ... + xn / n
        // Así, media [x2, x3, ..., xn, x(n+1)] := media [x1, x2, x3, ..., xn] - x1 / n + x(n+1) / n
        public systemdrawing.Point Suavizar(systemdrawing.Point p)
        {
            if (anteriores.Count < TAM_COLA)
            { // Si la cola es demasiado pequeña, meter valores pero devolver el punto recibido
                media.X += (p.X / TAM_COLA);
                media.Y += (p.Y / TAM_COLA);
                anteriores.Enqueue(p);
                return p;
            }
            else
            { // Si es suficiente, meter el nuevo, sacar el más antiguo, calcular el promedio y devolverlo
                systemdrawing.Point tmp = (systemdrawing.Point)anteriores.Dequeue();
                media.X -= (tmp.X / TAM_COLA);
                media.Y -= (tmp.Y / TAM_COLA);

                media.X += (p.X / TAM_COLA);
                media.Y += (p.Y / TAM_COLA);
                anteriores.Enqueue(p);
                return media;
            }
        }


        // Obtiene la zona de la MUÑECA DERECHA respecto del pecho, según cómo de delante o atrás esté
        // Hay 5 posibilidades de zonas:
        // - las zonas 2 y 4 son de seguridad (no mueve ni hace click).
        // - la zona 1 provoca click secundario
        // - la zona 3 provoca movimiento libre
        // - la zona 5 antigualmente provocaba click primario, ahora no hace nada
        public int getZona(Skeleton skel)
        {
            double pecho = skel.Joints[JointType.ShoulderCenter].Position.Z;
            double mano = skel.Joints[JointType.WristRight].Position.Z;

            if (mano > pecho) // Mano más atrás que el pecho
                return 1;
            else if (mano <= pecho && mano > pecho - 0.1) // Mano a 0.1 del pecho
                return 2;
            else if (mano <= pecho - 0.1 && mano > pecho - 0.4) // Mano a 0.1 - 0.4 del pecho
                return 3;
            else if (mano <= pecho - 0.4 && mano > pecho - 0.5) // Mano a 0.4 - 0.5 del pecho
                return 4;
            else // Mano a más de 0.5 del pecho
                return 5;
        }


        // Igual que getZona(Skeleton), pero en vez de ser de la muñeca derecha, es del parámentro Joint jt
        public int getZona(Skeleton skel, Joint jt)
        {
            double pecho = skel.Joints[JointType.ShoulderCenter].Position.Z;
            double mano = jt.Position.Z;

            if (mano > pecho) // Mano más atrás que el pecho
                return 1;
            else if (mano <= pecho && mano > pecho - 0.1) // Mano a 0.1 del pecho
                return 2;
            else if (mano <= pecho - 0.1 && mano > pecho - 0.4) // Mano a 0.1 - 0.4 del pecho
                return 3;
            else if (mano <= pecho - 0.4 && mano > pecho - 0.5) // Mano a 0.4 - 0.5 del pecho
                return 4;
            else // Mano a más de 0.5 del pecho
                return 5;
        }


        // Recibe un SKELETON y de él obtiene el JOINT de la MUÑECA DERECHA
        // Con una regla de 3, pasa las coordenadas X Y del joint a coordenadas X Y de la pantalla
        public systemdrawing.Point ManoToPantalla(Skeleton skel)
        {
            double x = skel.Joints[JointType.WristRight].Position.X;
            double y = skel.Joints[JointType.WristRight].Position.Y;
            int a, b;

            if (x < x1) a = 0;
            else if (x > x2) a = anchura;
            else a = (int)((double)anchura * ((x - x1) / (x2 - x1)));

            if (y > y1) b = 0;
            else if (y < y2) b = altura;
            else b = (int)((double)altura * ((y - y1) / (y2 - y1)));

            return new systemdrawing.Point(a, b); // Es importante que sea el Point definido en la librería system.drawing. En otro caso, no funcionará
        }


        // Es la primera función que se llama. Llamará a normal o diapositivas, mantiene (mal) el modo bloqueo y permite el cambio de modo
        public void General(Skeleton skel)
        {
            if (encendido)
            {
                if (!diapositiva)
                    ModoNormal(skel);
                else
                    ModoDiapositiva(skel);
            }

            if (skel.Joints[JointType.HandLeft].Position.Y > skel.Joints[JointType.Head].Position.Y)
            { // Si el joint de la MANO IZQUIERDA está sobre la CABEZA, se pone SOBRECABEZA = true (cuando se baje de por encima de la cabeza, se cambiará el modo)
                sobrecabeza = true;
            }
            else if (skel.Joints[JointType.HandLeft].Position.Y > skel.Joints[JointType.ElbowLeft].Position.Y && !diapositiva)
            { // ELSE, si el joint de la MANO IZQUIERDA está por encima de su codo y el modo diapositiva está apagado
                bloquear = true; // Modo de bloqueo activado
                setModo(2); // Enviar a setModo el nuevo modo de bloqueo para que ponga el color adecuado en el recuadro
            }
            else
            { // Si no se da la condición anterior
                if (sobrecabeza)
                { // Si anteriormente la mano estuvo sobre la cabeza (o sea, ya se ha bajado, porque si no sería el primer if y no este)
                    if (encendido && !diapositiva)
                    { // Cambiar de modo (1/2 -> 3)
                        encendido = false;
                        diapositiva = false;
                    }
                    else if (!encendido && !diapositiva)
                    { // Cambiar de modo (3 -> 4)
                        encendido = true;
                        diapositiva = true;

                    }
                    else if (encendido && diapositiva)
                    { // Cambiar de modo (4 -> 1/2)
                        encendido = true;
                        diapositiva = false;
                    }
                    sobrecabeza = false; // Como ya se ha realizado el cambio de modo, sobrecabeza se pone de nuevo a false
                }

                // Establece los modos (como o la mano está por debajo del codo o está en modo diapositiva, no puede estar en modo bloqueo (modo 2))
                bloquear = false;

                if (encendido)
                { // Establece los modos numéricos para todos los casos con setMode(int), para que el recuadro sea del color correcto.
                    if (diapositiva) setModo(4);
                    else if (bloquear) setModo(2);
                    else setModo(1);
                }
                else setModo(3);
            }
        }


        // Modo diapositiva (o modo 3)
        public void ModoDiapositiva(Skeleton skel)
        {
            // Aumenta contador de tiempo si alguna mano está cerrada, o lo pone a 0 si ha dejado de estarlo
            if (manoderechacerrada) contador_manoderechacerrada++;
            else contador_manoderechacerrada = 0;

            if (manoizquierdacerrada) contador_manoizquierdacerrada++;
            else contador_manoizquierdacerrada = 0;

            if (contador_manoderechacerrada == TIEMPO_CERRADO)
            { // Si el contador de mano derecha ha llegado al tiempo límite, enviar la tecla de PGDN (página abajo, o Av Pág)
                try
                { // El envío se consigue con SendKeys, que lo envía a la aplicación que tenga el foco. Puede dar problemas, por eso el try-catch
                    SendKeys.SendWait("{PGDN}"); // Avanzar
                }
                catch { }
                contador_manoderechacerrada = 0; // Reinicia los contadores
                contador_manoizquierdacerrada = 0;
            }
            else if (contador_manoizquierdacerrada == TIEMPO_CERRADO)
            { // Igual con mano izquierda
                try
                { // En este caso se envía página arriba, o Re Pág
                    SendKeys.SendWait("{PGUP}"); // Retroceder
                }
                catch { }
                contador_manoizquierdacerrada = 0;
                contador_manoderechacerrada = 0;
            }
        }

        // Modo normal (o modo 1) y bloqueo (modo 2)
        public void ModoNormal(Skeleton skel)
        {
            // Obtiene la zona en la que está la muñeca derecha (detrás, delante, muy delante del pecho....: hay 5 opciones)
            int actual = getZona(skel);

            if (contador_manoderechacerrada == 0)
            { // Si no ha empezado a agarrar, es decir, la mano está abierta
                if (actual == 1)
                { // Si la zona en la que está la mano es detrás del pecho, zona 1
                    if (anterior != actual) // Y en el frame justo anterior NO estaba en la misma zona (evita que se envíen cientos de clicks seguidos)
                        Click_botonsecundario(null, null); // Hacer click secundario (derecho)
                }
                else if (manoderechacerrada)
                { // Si antes la mano estaba cerrada (contador_manoderechacerrada == 0), pero ahora está abierta
                    contador_manoderechacerrada++; // empezar a contar (ahora mismo será contador_manoderechacerrada == 1)
                }
                else if (actual == 3)
                { // Si está en la zona 3 (lijeramente alejado del pecho, pero no demasiado)
                    if (!bloquear) // Y no está en modo bloqueo
                        Mover(Suavizar(ManoToPantalla(skel))); // MOVER con la posición suavizada
                }
            }
            else if (contador_manoderechacerrada > 0 && contador_manoderechacerrada < TIEMPO_CERRADO)
            { // Si no es el primer frame que está cerrado, pero tampoco ha llegado al límite del contador
                if (manoderechacerrada == false)
                { // SI HA DEJADO DE ESTAR CERRADO, poner a 0 y hacer un click (pulsar y soltar, no arrastrar, porque no ha llegado el tiempo suficiente)
                    contador_manoderechacerrada = 0; // Contador a 0
                    Click_botonprimario(null, null);
                }
                else
                { // Si sigue cerrado
                    contador_manoderechacerrada++; // Seguir contando
                }
            }
            else if (contador_manoderechacerrada == TIEMPO_CERRADO)
            { // Si no solo es el primer frame que está cerrado, sino que además ha llegado al límite del contador
                contador_manoderechacerrada++; 
                Mantener_botonprimario(null, null); // HACER CLICK Y MANTENER, NO SOLTAR
            }
            else if (contador_manoderechacerrada > TIEMPO_CERRADO)
            { // Si sigue más allá del límite
                if (manoderechacerrada) // Mientras mantenga la mano derecha cerrada, permitir moverse y arrastrar
                    Mover(Suavizar(ManoToPantalla(skel)));
                else
                { // Si ha soltado, poner contador a 0 y soltar click
                    Soltar_botonprimario(null, null);
                    contador_manoderechacerrada = 0;
                }
            }
            // Almacena el actual como el anterior, para no repetir acciones en el siguiente frame
            anterior = actual;
        }


        // Recibe un modo (1,2,3,4) y pone el cuadradito con el texto y color adecuado
        // 1: normal (movimiento y agarre). 2: block (movimiento bloqueado). 3: apagado. 4: diapositiva
        public void setModo(int modo)
        {
            if (this.modo != modo)
            { // Si el modo que ha recibido es el que ya está, no hace nada
                switch (modo)
                {
                    case 1:
                        lblModo.Content = "Ok";
                        lblModo.Background = new SolidColorBrush(Colors.Green);
                        break;
                    case 2:
                        lblModo.Content = "Block";
                        lblModo.Background = new SolidColorBrush(Colors.Yellow);
                        break;
                    case 3:
                        lblModo.Content = "Off";
                        lblModo.Background = new SolidColorBrush(Colors.Red);
                        break;
                    case 4:
                        lblModo.Content = "Diapositivas";
                        lblModo.Background = new SolidColorBrush(Colors.Blue);
                        break;
                }
                this.modo = modo;
            }
        }

            
        // Función que se llama cuando el ratón DEJA DE ESTAR en la palanca
        private void modificar_altura(object sender, System.Windows.Input.MouseEventArgs e)
        {
            try
            {
                if (sensor.ElevationAngle != (int)alturaslider.Value){ // Si ha cambiado
                    sensor.ElevationAngle = (int)alturaslider.Value; // Cambiar
                }
            }
            catch { }
        }


        // Función que se llama cuando se hace doble click en el dibujito
        // Cambia artificialmente de modos sin levantar la mano
        private void cambioModo(object sender, MouseButtonEventArgs e)
        {
            if (modo == 1 || modo == 2)
            { // Modo 1 (normal) y 2 (bloqueo) --> modo 3 (apagado)
                encendido = false;
                diapositiva = false;
                setModo(3);
            }
            else if (modo == 3)
            { // Modo 3 (apagado) --> modo 4 (diapositivas)
                encendido = true;
                diapositiva = true;
                setModo(4);
            }
            else if (modo == 4)
            { // Modo 4 (diapositivas) --> modo 1 (normal) y 2 bloqueo
                encendido = true;
                diapositiva = false;
                setModo(1);
            }
        }


    }
}