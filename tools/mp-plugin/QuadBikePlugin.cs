// QuadBike Mission Planner plugin — v1.4: nav-header readout, reads EFI_STATUS from comp 25.
//  - Subscribes to EFI_STATUS and reads GEAR (engine_load) / RPM (rpm) / TMP
//    (cylinder_head_temperature) from component 25 (the ESP32). One packet carries all three
//    as distinct fields, so values never override each other. customfields are NOT used.
//  - Hidden on start; reveals the first time comp 25 is heard, then NEVER hides (signal loss
//    keeps the last values on screen). "Hide if no MAVLink data with component 25."
//  - Fills the header height; drag moves it horizontally only (Y locked to 0); X persisted.
//  - Gear strip R N H L: sliding/blended/pulsing highlight reflects midpoint values
//    (0.5/1.5), both adjacent letters brighten, arrow under the active gear, pop on settle.
//  - RPM + color-coded TMP inline (blue <65, white 65-102, red >102).
// C# 5 compatible.
using System;
using System.IO;
using System.Text;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Plugin;

namespace QuadBike
{
    public class QbReadout : Control
    {
        private readonly string[] gears = new string[] { "R", "N", "H", "L" };
        private readonly Color[] cellCol = new Color[]
        {
            Color.FromArgb(230, 160, 40),   // R amber
            Color.FromArgb(90, 150, 255),   // N blue
            Color.FromArgb(60, 200, 90),    // H green
            Color.FromArgb(60, 200, 90)     // L green
        };

        public double GearValue = double.NaN;
        public double Rpm = double.NaN;
        public double Coolant = double.NaN;
        public bool Lock = false;    // front-wheel lock engaged
        public bool Light = false;   // front light on

        private double animIndex = 1.0;
        private double pulse = 0.0;
        private bool prevMoving = false;
        private DateTime popStart = DateTime.MinValue;
        private readonly Timer timer;

        private bool dragging;
        private int dragOffX;
        public const string PosFile = @"C:\Users\Public\quadbike_pos.txt";

        private const int padX = 6, top = 2, cellW = 36, cellH = 30;

        public QbReadout()
        {
            SetStyle(ControlStyles.AllPaintingInWmPaint | ControlStyles.UserPaint
                     | ControlStyles.OptimizedDoubleBuffer | ControlStyles.ResizeRedraw, true);
            Size = new Size(padX + 4 * cellW + 12 + 102 + 74, 47);
            Cursor = Cursors.SizeWE;   // horizontal move
            timer = new Timer();
            timer.Interval = 33;
            timer.Tick += OnTick;
            timer.Start();
        }

        private static double Rnd(double v) { return Math.Floor(v + 0.5); }
        private static int Li(int a, int b, double t) { return (int)(a + (b - a) * t); }
        private static Color LerpC(Color a, Color b, double t)
        {
            if (t < 0) t = 0; if (t > 1) t = 1;
            return Color.FromArgb(Li(a.R, b.R, t), Li(a.G, b.G, t), Li(a.B, b.B, t));
        }

        // horizontal-only drag (Y stays 0)
        protected override void OnMouseDown(MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Left) { dragging = true; dragOffX = e.X; }
            base.OnMouseDown(e);
        }
        protected override void OnMouseMove(MouseEventArgs e)
        {
            if (dragging)
            {
                int nx = Location.X + (e.X - dragOffX);
                if (nx < 0) nx = 0;
                if (Parent != null && nx > Parent.Width - Width) nx = Math.Max(0, Parent.Width - Width);
                Location = new Point(nx, 0);
            }
            base.OnMouseMove(e);
        }
        protected override void OnMouseUp(MouseEventArgs e)
        {
            if (dragging) { dragging = false; try { File.WriteAllText(PosFile, Location.X + ",0"); } catch { } }
            base.OnMouseUp(e);
        }

        private void OnTick(object s, EventArgs e)
        {
            if (!Visible) return;   // don't animate/repaint while hidden
            double target = double.IsNaN(GearValue) ? animIndex : (GearValue + 1.0);
            if (target < 0) target = 0; if (target > 3) target = 3;
            animIndex += (target - animIndex) * 0.35;
            pulse += 0.3;
            bool moving = !double.IsNaN(GearValue) && Math.Abs(GearValue - Rnd(GearValue)) > 0.1;
            if (prevMoving && !moving) popStart = DateTime.Now;
            prevMoving = moving;
            Invalidate();
        }

        private static GraphicsPath Rounded(RectangleF r, int rad)
        {
            GraphicsPath p = new GraphicsPath();
            float d = rad * 2;
            p.AddArc(r.X, r.Y, d, d, 180, 90);
            p.AddArc(r.Right - d, r.Y, d, d, 270, 90);
            p.AddArc(r.Right - d, r.Bottom - d, d, d, 0, 90);
            p.AddArc(r.X, r.Bottom - d, d, d, 90, 90);
            p.CloseFigure();
            return p;
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            Graphics g = e.Graphics;
            g.SmoothingMode = SmoothingMode.AntiAlias;
            g.Clear(Color.FromArgb(40, 40, 46));

            bool valid = !double.IsNaN(GearValue);
            bool moving = valid && Math.Abs(GearValue - Rnd(GearValue)) > 0.1;

            int lo = (int)Math.Floor(animIndex); if (lo < 0) lo = 0; if (lo > 3) lo = 3;
            int hi = lo + 1; if (hi > 3) hi = 3;
            double frac = animIndex - lo; if (frac < 0) frac = 0; if (frac > 1) frac = 1;
            Color hc = LerpC(cellCol[lo], cellCol[hi], frac);

            double pop = 1.0;
            double sinceMs = (DateTime.Now - popStart).TotalMilliseconds;
            if (sinceMs < 250) pop = 1.0 + 0.18 * (1.0 - sinceMs / 250.0);

            if (valid)
            {
                double cx = padX + animIndex * cellW + cellW / 2.0;
                double cy = top + cellH / 2.0;
                int alpha = moving ? (int)(150 + 80 * Math.Sin(pulse)) : 230;
                if (alpha < 60) alpha = 60; if (alpha > 255) alpha = 255;
                double hw = (cellW - 6) * pop, hh = (cellH - 4) * pop;
                RectangleF hr = new RectangleF((float)(cx - hw / 2), (float)(cy - hh / 2), (float)hw, (float)hh);
                using (GraphicsPath gp = Rounded(hr, 6))
                using (SolidBrush b = new SolidBrush(Color.FromArgb(alpha, hc)))
                    g.FillPath(b, gp);
            }

            using (Font f = new Font("Tahoma", 17, FontStyle.Bold))
            {
                StringFormat sf = new StringFormat();
                sf.Alignment = StringAlignment.Center;
                sf.LineAlignment = StringAlignment.Center;
                for (int i = 0; i < 4; i++)
                {
                    double closeness = 1.0 - Math.Abs(i - animIndex);
                    if (closeness < 0) closeness = 0;
                    Color lc = valid ? LerpC(Color.FromArgb(110, 110, 116), Color.White, closeness)
                                     : Color.FromArgb(110, 110, 116);
                    RectangleF cell = new RectangleF(padX + i * cellW, top, cellW, cellH);
                    using (SolidBrush tb = new SolidBrush(lc))
                        g.DrawString(gears[i], f, tb, cell, sf);
                }
            }

            if (valid)
            {
                double cx = padX + animIndex * cellW + cellW / 2.0;
                float ay = top + cellH + 1;
                PointF[] tri = new PointF[]
                {
                    new PointF((float)cx, ay),
                    new PointF((float)cx - 7, ay + 7),
                    new PointF((float)cx + 7, ay + 7)
                };
                using (SolidBrush ab = new SolidBrush(hc))
                    g.FillPolygon(ab, tri);
            }

            // RPM (top) / TMP (bottom), larger, inline to the right
            using (Font f2 = new Font("Tahoma", 11f, FontStyle.Bold))
            {
                float tx = padX + 4 * cellW + 12;
                string rpmTxt = double.IsNaN(Rpm) ? "RPM --" : ("RPM " + Rpm.ToString("0"));
                using (SolidBrush rb = new SolidBrush(Color.FromArgb(215, 215, 220)))
                    g.DrawString(rpmTxt, f2, rb, tx, 3);

                string tmpTxt = double.IsNaN(Coolant) ? "TMP --" : ("TMP " + Coolant.ToString("0") + "°");
                // 65..102 = white (normal); below 65 = blue (cold); above 102 = red (hot)
                Color cc = Color.White;
                if (!double.IsNaN(Coolant)) { if (Coolant < 65) cc = Color.FromArgb(90, 160, 255); else if (Coolant > 102) cc = Color.FromArgb(220, 60, 60); }
                using (SolidBrush cb = new SolidBrush(cc))
                    g.DrawString(tmpTxt, f2, cb, tx, 25);

                // Digital-state badges (second column, row-aligned with RPM/TMP):
                //   LOCK  — red when the front-wheel lock is engaged, dim grey otherwise
                //   LIGHT — amber when the front light is on, dim grey otherwise
                float bx = tx + 100;
                Color dim = Color.FromArgb(105, 105, 112);
                using (SolidBrush lk = new SolidBrush(Lock  ? Color.FromArgb(220, 60, 60)  : dim))
                    g.DrawString("LOCK", f2, lk, bx, 3);
                using (SolidBrush lt = new SolidBrush(Light ? Color.FromArgb(255, 165, 0) : dim))
                    g.DrawString("LIGHT", f2, lt, bx, 25);
            }
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing && timer != null) { timer.Stop(); timer.Dispose(); }
            base.Dispose(disposing);
        }
    }

    public class QuadBikePlugin : MissionPlanner.Plugin.Plugin
    {
        public override string Name { get { return "QuadBike Telemetry"; } }
        public override string Version { get { return "1.4-efi"; } }
        public override string Author { get { return "QuadBikeControl"; } }
        public override float loopratehz { get { return 5f; } }

        private const byte ESP32_SYSID = 1;     // MAVLINK_SYSTEM_ID (same as autopilot)
        private const byte ESP32_COMPID = 25;    // MAVLINK_COMPONENT_ID

        private QbReadout view;
        private readonly object sync = new object();
        private double nvGear = double.NaN, nvRpm = double.NaN, nvTmp = double.NaN;
        private bool nvLock = false, nvLight = false;
        private volatile bool sawComp25 = false;

        // Digital-flag bit map (EFI_STATUS.pt_compensation) — MUST match EFI_DIGITAL_FLAG_* in firmware Constants.h
        private const int FLAG_WHEEL_LOCK = 0x01;   // bit0
        private const int FLAG_FRONT_LIGHT = 0x02;  // bit1
        private bool revealed = false;
        private int sub = 0;            // NVF subscription handle (0 = not subscribed)
        private int nvfLogCount = 0;

        private static void Log(string s)
        {
            try { File.AppendAllText(@"C:\Users\Public\quadbike_plugin.log",
                DateTime.Now.ToString("HH:mm:ss.fff") + "  " + s + Environment.NewLine); } catch { }
        }

        public override bool Init() { Log("Init v1.4-efi"); return true; }

        public override bool Loaded()
        {
            try
            {
                Host.MainForm.BeginInvoke((MethodInvoker)delegate
                {
                    try { Build(); } catch (Exception ex) { Log("Build EX: " + ex); }
                });
                // subscription is (re)established in Loop() once the link is open
            }
            catch (Exception ex) { Log("Loaded EX: " + ex); }
            return true;
        }

        // (Re)establish the EFI_STATUS subscription once the link is open; drop it when closed.
        // MP subscriptions don't survive a connect, so we manage it from Loop() like the
        // bundled generator.cs plugin does.
        private void EnsureSubscription()
        {
            bool open = false;
            try { open = MainV2.comPort.BaseStream.IsOpen || MainV2.comPort.logreadmode; } catch { }

            if (open && sub == 0)
            {
                try
                {
                    // One EFI_STATUS packet carries all three values as distinct fields, so
                    // nothing can override anything: engine_load=GEAR, rpm=RPM, cyl head temp=TMP.
                    sub = MainV2.comPort.SubscribeToPacketType(MAVLink.MAVLINK_MSG_ID.EFI_STATUS, message =>
                    {
                        try
                        {
                            if (message.compid != ESP32_COMPID) return true;   // only the ESP32
                            MAVLink.mavlink_efi_status_t efi = (MAVLink.mavlink_efi_status_t)message.data;
                            double gear = efi.engine_load;
                            double rpm = efi.rpm;
                            double tmp = efi.cylinder_head_temperature;
                            int flags = (int)Math.Round((double)efi.pt_compensation);   // digital-state bitmask
                            bool lk = (flags & FLAG_WHEEL_LOCK) != 0;
                            bool lt = (flags & FLAG_FRONT_LIGHT) != 0;
                            lock (sync)
                            {
                                nvGear = gear;
                                nvRpm = rpm;     // firmware sends NaN while CAN invalid -> "--"
                                nvTmp = tmp;
                                nvLock = lk;
                                nvLight = lt;
                            }
                            sawComp25 = true;
                            if (nvfLogCount < 10) { Log("EFI comp" + message.compid + " gear=" + gear + " rpm=" + rpm + " cht=" + tmp + " flags=" + flags); nvfLogCount++; }
                        }
                        catch (Exception ex) { Log("EFI cb EX: " + ex.Message); }
                        return true;   // stay subscribed
                    }, ESP32_SYSID, ESP32_COMPID);   // exact match: fires even when comp 1 is the primary MAV
                    Log("subscribed to EFI_STATUS sys" + ESP32_SYSID + "/comp" + ESP32_COMPID + " (sub=" + sub + ")");
                }
                catch (Exception ex) { Log("Subscribe EX: " + ex); }
            }
            else if (!open && sub != 0)
            {
                try { MainV2.comPort.UnSubscribeToPacketType(sub); } catch { }
                sub = 0;
                Log("unsubscribed (link closed)");
            }
        }

        private static T FindControl<T>(Control root) where T : class
        {
            foreach (Control c in root.Controls)
            {
                T hit = c as T;
                if (hit != null) return hit;
                T r = FindControl<T>(c);
                if (r != null) return r;
            }
            return null;
        }

        private static int LoadX()
        {
            try
            {
                if (File.Exists(QbReadout.PosFile))
                {
                    string[] a = File.ReadAllText(QbReadout.PosFile).Split(',');
                    return int.Parse(a[0].Trim());
                }
            }
            catch { }
            return 330;   // default X — just right of the HELP button (HELP ends at x=324)
        }

        private void Build()
        {
            MenuStrip ms = FindControl<MenuStrip>(Host.MainForm);
            if (ms == null) { Log("MenuStrip not found"); return; }
            Control header = ms.Parent;
            if (header == null) { Log("header (panel1) null"); return; }

            view = new QbReadout();
            view.Visible = false;            // hidden until comp 25 is heard
            header.Controls.Add(view);
            view.Height = header.Height;     // full header height
            int x = LoadX();
            int maxX = Math.Max(0, header.Width - view.Width);
            if (x < 0) x = 0; if (x > maxX) x = maxX;
            view.Location = new Point(x, 0);  // Y locked to 0 (no vertical movement)
            view.BringToFront();
            Log("QbReadout added (hidden); header=" + header.Size + " loc=" + view.Location + " size=" + view.Size);
        }

        public override bool Loop()
        {
            try
            {
                EnsureSubscription();
                if (view == null) return true;

                double g, r, c;
                bool lk, lt;
                lock (sync) { g = nvGear; r = nvRpm; c = nvTmp; lk = nvLock; lt = nvLight; }
                bool seen = sawComp25;

                Host.MainForm.BeginInvoke((MethodInvoker)delegate
                {
                    view.GearValue = g; view.Rpm = r; view.Coolant = c;
                    view.Lock = lk; view.Light = lt;
                    if (seen && !view.Visible) { view.Visible = true; view.BringToFront(); }
                });

                if (seen && !revealed) { revealed = true; Log("comp25 seen -> readout shown"); }
            }
            catch (Exception ex) { Log("Loop EX: " + ex.Message); }
            return true;
        }

        public override bool Exit() { return true; }
    }
}
