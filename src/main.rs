//! Expose a monitor's DDC/CI ambient light sensor as net.hadess.SensorProxy.
//!
//! KWin (and anything else that speaks iio-sensor-proxy's D-Bus API) picks the
//! reading up as a regular ambient light sensor.  Only the light sensor part of
//! the interface is implemented; accelerometer and proximity always report
//! "not present".
//!
//! The Dell U4025QW reports its sensor in VCP 0x67 while auto-brightness
//! (VCP 0x66 == 0x82) is enabled; the reading keeps working in HDR mode, where
//! the OSD greys the option out.
//!
//! To keep wakeups down the poll interval adapts: it starts at --interval and
//! doubles while the reading is steady, up to --max-interval while the sensor
//! is claimed and --idle-interval otherwise.  DRM hotplug uevents trigger an
//! immediate re-probe, so connecting the monitor doesn't wait for the idle poll.

use std::collections::HashMap;
use std::fs::{File, OpenOptions};
use std::io::{self, Read, Write};
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant};

use clap::Parser;
use futures_util::StreamExt;
use log::{debug, info, warn};
use tokio::io::unix::AsyncFd;
use tokio::sync::Notify;
use tokio::task::JoinHandle;
use zbus::object_server::{InterfaceRef, SignalEmitter};
use zbus::{connection, fdo, interface, message::Header, Connection};

const BUS_NAME: &str = "net.hadess.SensorProxy";
const OBJECT_PATH: &str = "/net/hadess/SensorProxy";

const I2C_SLAVE: libc::c_ulong = 0x0703;
const DDC_ADDR: libc::c_ulong = 0x37;

/// How often to re-read the on/off switch while the sensor reads fine.
const ENABLE_RECHECK: Duration = Duration::from_secs(60);

fn parse_int(s: &str) -> Result<u8, String> {
    let r = match s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        Some(hex) => u8::from_str_radix(hex, 16),
        None => s.parse(),
    };
    r.map_err(|e| e.to_string())
}

#[derive(Parser, Debug)]
#[command(about = "Expose a monitor's DDC/CI ambient light sensor as net.hadess.SensorProxy")]
struct Args {
    /// Substring of the monitor's EDID name
    #[arg(long, default_value = "U4025QW")]
    model: String,
    /// i2c device to use instead of looking up --model
    #[arg(long)]
    device: Option<PathBuf>,
    /// VCP code holding the sensor reading
    #[arg(long, value_parser = parse_int, default_value = "0x67")]
    vcp: u8,
    /// VCP code of the sensor on/off switch
    #[arg(long, value_parser = parse_int, default_value = "0x66")]
    enable_vcp: u8,
    /// Subtract from the raw reading
    #[arg(long, default_value_t = 0.0)]
    offset: f64,
    /// Multiply the offset reading to get lux
    #[arg(long, default_value_t = 1.0)]
    scale: f64,
    /// Shortest poll interval in seconds, used right after a change
    #[arg(long, default_value_t = 2.0)]
    interval: f64,
    /// Longest poll interval in seconds while claimed and the reading is steady
    #[arg(long, default_value_t = 10.0)]
    max_interval: f64,
    /// Poll interval in seconds while unclaimed or the sensor is unavailable
    #[arg(long, default_value_t = 60.0)]
    idle_interval: f64,
    /// Use the session bus (for testing)
    #[arg(long)]
    session: bool,
    #[arg(short, long)]
    verbose: bool,
}

// --- DDC/CI ---

fn edid_model(edid: &[u8]) -> Option<String> {
    for off in [54, 72, 90, 108] {
        let Some(desc) = edid.get(off..off + 18) else { continue };
        if desc[..3] == [0, 0, 0] && desc[3] == 0xFC {
            let name = desc[5..].split(|&b| b == b'\n').next().unwrap_or_default();
            return Some(String::from_utf8_lossy(name).trim().to_string());
        }
    }
    None
}

/// Return /dev/i2c-N of the connected DRM connector whose EDID name contains model.
fn find_i2c_bus(model: &str) -> Option<PathBuf> {
    let mut conns: Vec<_> = std::fs::read_dir("/sys/class/drm")
        .ok()?
        .flatten()
        .map(|e| e.path())
        .filter(|p| {
            let n = p.file_name().unwrap_or_default().to_string_lossy();
            n.starts_with("card") && n.contains('-')
        })
        .collect();
    conns.sort();
    for conn in conns {
        let connected = std::fs::read_to_string(conn.join("status"))
            .is_ok_and(|s| s.trim() == "connected");
        if !connected {
            continue;
        }
        let Ok(edid) = std::fs::read(conn.join("edid")) else { continue };
        if !edid_model(&edid).is_some_and(|n| n.contains(model)) {
            continue;
        }
        // DP connectors carry their AUX i2c adapter as a child; HDMI/DVI use the ddc link.
        let mut candidates: Vec<String> = std::fs::read_dir(&conn)
            .into_iter()
            .flatten()
            .flatten()
            .map(|e| e.file_name().to_string_lossy().into_owned())
            .filter(|n| n.starts_with("i2c-"))
            .collect();
        candidates.sort();
        if candidates.is_empty() {
            if let Ok(ddc) = std::fs::canonicalize(conn.join("ddc")) {
                candidates.extend(ddc.file_name().map(|n| n.to_string_lossy().into_owned()));
            }
        }
        if let Some(c) = candidates.first() {
            return Some(Path::new("/dev").join(c));
        }
    }
    None
}

struct Ddc {
    file: File,
}

impl Ddc {
    fn open(dev: &Path) -> io::Result<Self> {
        let file = OpenOptions::new().read(true).write(true).open(dev)?;
        // SAFETY: plain ioctl on an fd we own.
        if unsafe { libc::ioctl(file.as_raw_fd(), I2C_SLAVE as _, DDC_ADDR) } < 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(Self { file })
    }

    /// Return (current, max) for a VCP feature.
    async fn get_vcp(&mut self, code: u8) -> io::Result<(u16, u16)> {
        let mut req = [0x51, 0x82, 0x01, code, 0];
        req[4] = req[..4].iter().fold(0x6E, |c, b| c ^ b);
        let mut err = None;
        for _ in 0..4 {
            match self.try_get_vcp(&req, code).await {
                Ok(v) => return Ok(v),
                Err(e) => err = Some(e),
            }
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
        Err(err.unwrap())
    }

    async fn try_get_vcp(&mut self, req: &[u8], code: u8) -> io::Result<(u16, u16)> {
        self.file.write_all(req)?;
        tokio::time::sleep(Duration::from_millis(50)).await;
        let mut resp = [0u8; 11];
        let n = self.file.read(&mut resp)?;
        let chk = resp[..10].iter().fold(0x50, |c, b| c ^ b);
        if n != 11 || resp[1] != 0x88 || resp[2] != 0x02 || resp[4] != code || chk != resp[10] {
            return Err(io::Error::other(format!("bad DDC reply {:02x?}", &resp[..n])));
        }
        if resp[3] != 0 {
            return Err(io::Error::other(format!("VCP 0x{code:02x} unsupported")));
        }
        let cur = u16::from_be_bytes([resp[8], resp[9]]);
        let max = u16::from_be_bytes([resp[6], resp[7]]);
        Ok((cur, max))
    }
}

struct Sensor {
    args: Args,
    ddc: Option<Ddc>,
    enable_checked: Option<Instant>,
}

impl Sensor {
    fn reset(&mut self) {
        self.ddc = None;
        self.enable_checked = None;
    }

    async fn read(&mut self) -> Option<f64> {
        if self.ddc.is_none() {
            let dev = self.args.device.clone().or_else(|| find_i2c_bus(&self.args.model));
            let Some(dev) = dev else {
                debug!("monitor {:?} not connected", self.args.model);
                return None;
            };
            match Ddc::open(&dev) {
                Ok(d) => {
                    info!("using {}", dev.display());
                    self.ddc = Some(d);
                }
                Err(e) => {
                    warn!("cannot open {}: {e}", dev.display());
                    return None;
                }
            }
        }
        match self.read_ddc().await {
            Ok(v) => v,
            Err(e) => {
                warn!("DDC read failed: {e}");
                self.reset();
                None
            }
        }
    }

    async fn read_ddc(&mut self) -> io::Result<Option<f64>> {
        let ddc = self.ddc.as_mut().unwrap();
        // The switch rarely changes; only re-read it now and then.
        if self.enable_checked.is_none_or(|t| t.elapsed() >= ENABLE_RECHECK) {
            let (enabled, _) = ddc.get_vcp(self.args.enable_vcp).await?;
            if enabled & 0x7F != 0x02 {
                debug!("sensor disabled (VCP 0x{:02x} = 0x{enabled:02x})", self.args.enable_vcp);
                self.enable_checked = None;
                return Ok(None);
            }
            self.enable_checked = Some(Instant::now());
        }
        let (raw, _) = ddc.get_vcp(self.args.vcp).await?;
        Ok(Some(((raw as f64 - self.args.offset) * self.args.scale).max(0.0)))
    }
}

// --- DRM hotplug ---

fn uevent_socket() -> io::Result<AsyncFd<OwnedFd>> {
    // SAFETY: plain socket()/bind() calls; the fd is wrapped immediately.
    unsafe {
        let fd = libc::socket(
            libc::AF_NETLINK,
            libc::SOCK_DGRAM | libc::SOCK_CLOEXEC | libc::SOCK_NONBLOCK,
            libc::NETLINK_KOBJECT_UEVENT,
        );
        if fd < 0 {
            return Err(io::Error::last_os_error());
        }
        let fd = OwnedFd::from_raw_fd(fd);
        let mut addr: libc::sockaddr_nl = std::mem::zeroed();
        addr.nl_family = libc::AF_NETLINK as _;
        addr.nl_groups = 1; // kernel uevents
        if libc::bind(
            fd.as_raw_fd(),
            &addr as *const _ as *const libc::sockaddr,
            size_of::<libc::sockaddr_nl>() as _,
        ) < 0
        {
            return Err(io::Error::last_os_error());
        }
        // The fd moves into the AsyncFd, so it can't be closed while registered.
        AsyncFd::register(fd).map_err(|e| e.into_parts().1)
    }
}

async fn watch_hotplug(sock: AsyncFd<OwnedFd>, hotplug: Arc<Notify>) {
    let mut buf = [0u8; 8192];
    loop {
        let Ok(mut guard) = sock.readable().await else { return };
        // SAFETY: recv into a buffer we own.
        let n = unsafe {
            libc::recv(sock.get_ref().as_raw_fd(), buf.as_mut_ptr().cast(), buf.len(), 0)
        };
        if n < 0 {
            if io::Error::last_os_error().kind() == io::ErrorKind::WouldBlock {
                guard.clear_ready();
            }
            continue;
        }
        let drm = buf[..n as usize].split(|&b| b == 0).any(|kv| kv == b"SUBSYSTEM=drm");
        if drm {
            debug!("DRM uevent");
            hotplug.notify_one();
        }
    }
}

// --- net.hadess.SensorProxy ---

struct SensorProxy {
    has_ambient_light: bool,
    light_level: f64,
    claims: HashMap<String, JoinHandle<()>>,
    claimed: Arc<Notify>,
}

impl SensorProxy {
    fn release(&mut self, sender: &str) {
        if let Some(watch) = self.claims.remove(sender) {
            watch.abort();
            info!("light released by {sender}");
        }
    }
}

/// Drop sender's claim once it disconnects from the bus.
async fn watch_claimer(conn: Connection, sender: String) {
    let watch = async {
        let dbus = fdo::DBusProxy::new(&conn).await?;
        let mut gone = dbus
            .receive_name_owner_changed_with_args(&[(0, sender.as_str())])
            .await?;
        if dbus.name_has_owner(sender.as_str().try_into()?).await? {
            while let Some(sig) = gone.next().await {
                if sig.args()?.new_owner().is_none() {
                    break;
                }
            }
        }
        zbus::Result::Ok(())
    };
    if let Err(e) = watch.await {
        warn!("cannot watch {sender}: {e}");
    }
    let iface = conn.object_server().interface::<_, SensorProxy>(OBJECT_PATH).await;
    if let Ok(iface) = iface {
        iface.get_mut().await.release(&sender);
    }
}

#[interface(name = "net.hadess.SensorProxy")]
impl SensorProxy {
    async fn claim_light(
        &mut self,
        #[zbus(header)] hdr: Header<'_>,
        #[zbus(connection)] conn: &Connection,
    ) {
        let Some(sender) = hdr.sender().map(|s| s.to_string()) else { return };
        if !self.claims.contains_key(&sender) {
            info!("light claimed by {sender}");
            let watch = tokio::spawn(watch_claimer(conn.clone(), sender.clone()));
            self.claims.insert(sender, watch);
            self.claimed.notify_one();
        }
    }

    async fn release_light(&mut self, #[zbus(header)] hdr: Header<'_>) {
        if let Some(sender) = hdr.sender() {
            self.release(sender);
        }
    }

    async fn claim_accelerometer(&self) {}
    async fn release_accelerometer(&self) {}
    async fn claim_proximity(&self) {}
    async fn release_proximity(&self) {}

    #[zbus(property)]
    fn has_accelerometer(&self) -> bool {
        false
    }

    #[zbus(property)]
    fn accelerometer_orientation(&self) -> &str {
        "undefined"
    }

    #[zbus(property)]
    fn accelerometer_tilt(&self) -> &str {
        "undefined"
    }

    #[zbus(property)]
    fn has_ambient_light(&self) -> bool {
        self.has_ambient_light
    }

    #[zbus(property)]
    fn light_level_unit(&self) -> &str {
        "lux"
    }

    #[zbus(property)]
    fn light_level(&self) -> f64 {
        self.light_level
    }

    #[zbus(property)]
    fn has_proximity(&self) -> bool {
        false
    }

    #[zbus(property)]
    fn proximity_near(&self) -> bool {
        false
    }
}

// --- polling ---

/// Publish a reading; return whether it differs enough to poll faster again.
async fn publish(iface: &InterfaceRef<SensorProxy>, lux: Option<f64>, scale: f64) -> zbus::Result<bool> {
    let emitter: &SignalEmitter<'_> = iface.signal_emitter();
    let mut proxy = iface.get_mut().await;
    let mut significant = false;
    if proxy.has_ambient_light != lux.is_some() {
        proxy.has_ambient_light = lux.is_some();
        proxy.has_ambient_light_changed(emitter).await?;
        significant = true;
    }
    if let Some(lux) = lux.filter(|&l| l != proxy.light_level) {
        let old = proxy.light_level;
        // Sensor noise of a step or so shouldn't keep us polling fast.
        significant |= (lux - old).abs() > (old * 0.1).max(scale.abs());
        proxy.light_level = lux;
        proxy.light_level_changed(emitter).await?;
        debug!("light level {lux}");
    }
    Ok(significant)
}

async fn poll_loop(
    mut sensor: Sensor,
    iface: InterfaceRef<SensorProxy>,
    claimed: Arc<Notify>,
    hotplug: Arc<Notify>,
) {
    let min = Duration::from_secs_f64(sensor.args.interval);
    let max = Duration::from_secs_f64(sensor.args.max_interval).max(min);
    let idle = Duration::from_secs_f64(sensor.args.idle_interval).max(min);
    let mut delay = min;
    loop {
        tokio::select! {
            _ = tokio::time::sleep(delay) => {}
            _ = claimed.notified() => delay = min,
            _ = hotplug.notified() => {
                // The monitor may have moved or the DDC channel gone stale.
                if sensor.args.device.is_none() {
                    sensor.reset();
                }
                delay = min;
            }
        }
        let lux = sensor.read().await;
        let significant = match publish(&iface, lux, sensor.args.scale).await {
            Ok(s) => s,
            Err(e) => {
                warn!("cannot emit PropertiesChanged: {e}");
                false
            }
        };
        let active = lux.is_some() && !iface.get().await.claims.is_empty();
        let cap = if active { max } else { idle };
        delay = if significant { min } else { (delay * 2).min(cap) };
        debug!("next poll in {delay:?}");
    }
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> zbus::Result<()> {
    let args = Args::parse();
    env_logger::Builder::new()
        .filter_level(if args.verbose { log::LevelFilter::Debug } else { log::LevelFilter::Info })
        .format(|buf, rec| writeln!(buf, "{}: {}", rec.level(), rec.args()))
        .init();

    let session = args.session;
    let mut sensor = Sensor { args, ddc: None, enable_checked: None };
    // Determine HasAmbientLight before taking the bus name.
    let lux = sensor.read().await;

    let claimed = Arc::new(Notify::new());
    let hotplug = Arc::new(Notify::new());
    let proxy = SensorProxy {
        has_ambient_light: lux.is_some(),
        light_level: lux.unwrap_or(0.0),
        claims: HashMap::new(),
        claimed: claimed.clone(),
    };
    let builder = if session { connection::Builder::session()? } else { connection::Builder::system()? };
    let conn = builder.serve_at(OBJECT_PATH, proxy)?.name(BUS_NAME)?.build().await?;
    let iface = conn.object_server().interface::<_, SensorProxy>(OBJECT_PATH).await?;

    match uevent_socket() {
        Ok(sock) => drop(tokio::spawn(watch_hotplug(sock, hotplug.clone()))),
        Err(e) => warn!("cannot listen for hotplug events: {e}"),
    }
    poll_loop(sensor, iface, claimed, hotplug).await;
    Ok(())
}
