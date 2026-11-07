//! DM-MC02 (STM32H723VG) board setup: clocks, CAN1, DR16 input, watchdog and panic handling.
//!
//! The CPU runs at 520 MHz from the 24 MHz crystal. TIM2 is Embassy's time driver.

use core::fmt::Write as _;
use core::num::{NonZeroU8, NonZeroU16};
use core::panic::PanicInfo;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use embassy_stm32::can::config::NominalBitTiming;
use embassy_stm32::can::enums::BusError;
use embassy_stm32::can::{self, BufferedCan, CanConfigurator, RxBuf, TxBuf};
use embassy_stm32::dma;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::mode::Async;
use embassy_stm32::rcc::mux::Fdcansel;
use embassy_stm32::rcc::{
    AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllDiv, PllMul, PllPreDiv, PllSource, Sysclk,
    VoltageScale,
};
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{self, Parity, StopBits, UartRx};
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_stm32::{bind_interrupts, pac};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};

use static_cell::StaticCell;

pub type RemoteRx = UartRx<'static, Async>;
pub static REMOTE: Signal<CriticalSectionRawMutex, Option<Dr16>> = Signal::new();

pub type Can1 = BufferedCan<'static, 4, 16>;
static CAN_TX: StaticCell<TxBuf<4>> = StaticCell::new();
static CAN_RX: StaticCell<RxBuf<16>> = StaticCell::new();

/// CAN bus errors reported to `log_can_error` since startup.
pub static CAN_ERRORS: AtomicU32 = AtomicU32::new(0);

pub type Watchdog = IndependentWatchdog<'static, embassy_stm32::peripherals::IWDG1>;

/// Nominal; the LSI clock varies with voltage and temperature.
const WATCHDOG_TIMEOUT_US: u32 = 100_000;

bind_interrupts!(struct Irqs {
    UART5 => usart::InterruptHandler<embassy_stm32::peripherals::UART5>;
    DMA1_STREAM5 => dma::InterruptHandler<embassy_stm32::peripherals::DMA1_CH5>;
    FDCAN1_IT0 => can::IT0InterruptHandler<embassy_stm32::peripherals::FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<embassy_stm32::peripherals::FDCAN1>;
});

/// Take the MCU peripherals and return CAN1, the started watchdog and the DR16 receiver.
/// Call once at startup. Power the board from USB or its normal input: probe power alone
/// does not power the CAN transceivers.
pub fn init() -> (Can1, Watchdog, RemoteRx) {
    // After a watchdog reset, halt before CAN starts. Clearing the reset flags first lets
    // the next manual reset run normally.
    let watchdog_reset = pac::RCC.rsr().read().iwdg1rstf();
    pac::RCC.rsr().modify(|w| w.set_rmvf(true));
    if watchdog_reset {
        rtt_target::rprintln!("Watchdog reset: halted. Press RESET to run again.");
        cortex_m::interrupt::disable();
        loop {
            core::hint::spin_loop();
        }
    }

    // 24 MHz / 3 * 65 = 520 MHz, the VOS0 limit without the CPU boost option bit.
    // AHB runs at 260 MHz and every APB at 130 MHz. CAN takes the crystal directly.
    let mut config = embassy_stm32::Config::default();
    config.rcc.hse = Some(Hse {
        freq: Hertz(24_000_000),
        mode: HseMode::Oscillator,
    });
    config.rcc.pll1 = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV3,
        mul: PllMul::MUL65,
        divp: Some(PllDiv::DIV1),
        divq: None,
        divr: None,
    });
    config.rcc.sys = Sysclk::PLL1_P;
    config.rcc.ahb_pre = AHBPrescaler::DIV2;
    config.rcc.apb1_pre = APBPrescaler::DIV2;
    config.rcc.apb2_pre = APBPrescaler::DIV2;
    config.rcc.apb3_pre = APBPrescaler::DIV2;
    config.rcc.apb4_pre = APBPrescaler::DIV2;
    config.rcc.voltage_scale = VoltageScale::Scale0;
    config.rcc.mux.fdcansel = Fdcansel::HSE;
    let p = embassy_stm32::init(config);

    // Only the control task feeds this, after each completed tick.
    let mut watchdog = Watchdog::new(p.IWDG1, WATCHDOG_TIMEOUT_US);
    watchdog.unleash();

    // Classic CAN on PD0 (RX) and PD1 (TX): 24 MHz / (1 + 17 + 6) = 1 Mbit/s, sampled at 75%.
    let mut can = CanConfigurator::new(p.FDCAN1, p.PD0, p.PD1, Irqs);
    can.set_config(can.config().set_nominal_bit_timing(NominalBitTiming {
        prescaler: NonZeroU16::new(1).unwrap(),
        seg1: NonZeroU8::new(17).unwrap(),
        seg2: NonZeroU8::new(6).unwrap(),
        sync_jump_width: NonZeroU8::new(6).unwrap(),
    }));
    let can = can
        .into_normal_mode()
        .buffered(CAN_TX.init(TxBuf::new()), CAN_RX.init(RxBuf::new()));

    // DBUS on PD2: 100 kbaud, 8E2. The board inverts the signal, so the UART does not.
    let mut remote_config = usart::Config::default();
    remote_config.baudrate = 100_000;
    remote_config.parity = Parity::ParityEven;
    remote_config.stop_bits = StopBits::STOP2;
    let remote = UartRx::new(p.UART5, p.PD2, p.DMA1_CH5, Irqs, remote_config).unwrap();
    // PC15 switches the 5 V rail that powers the receiver.
    core::mem::forget(Output::new(p.PC15, Level::High, Speed::Low));
    (can, watchdog, remote)
}

/// Count a CAN bus error, and log the first one and every 1000th after it.
pub fn log_can_error(error: BusError) {
    let count = CAN_ERRORS.fetch_add(1, Ordering::Relaxed) + 1;
    if count == 1 || count % 1000 == 0 {
        rtt_target::rprintln!("CAN1 bus error {}: {:?}", count, error);
    }
}

/// One valid 18-byte DBUS frame, without remapping or deadzones.
#[derive(Clone, Copy)]
pub struct Dr16 {
    /// Right horizontal, right vertical, left horizontal, left vertical, wheel; -660..=660.
    pub channels: [i16; 5],
    /// 1 up, 2 down, 3 middle.
    pub right_switch: u8,
    pub left_switch: u8,
    pub mouse: [i16; 3],
    pub mouse_left: bool,
    pub mouse_right: bool,
    /// Bits 0..15: W S A D Shift Ctrl Q E R F G Z X C V B.
    pub keys: u16,
}

impl Dr16 {
    /// Returns `None` for partial frames and out-of-range channels or switches.
    pub fn parse(bytes: &[u8]) -> Option<Self> {
        let b: &[u8; 18] = bytes.try_into().ok()?;
        let u = |i: usize| u16::from(b[i]);
        let channels = [
            u(0) | u(1) << 8,
            u(1) >> 3 | u(2) << 5,
            u(2) >> 6 | u(3) << 2 | u(4) << 10,
            u(4) >> 1 | u(5) << 7,
            u(16) | u(17) << 8,
        ]
        .map(|value| (value & 0x7ff) as i16 - 1024);
        let right_switch = b[5] >> 4 & 3;
        let left_switch = b[5] >> 6 & 3;
        if channels.iter().any(|value| !(-660..=660).contains(value))
            || !(1..=3).contains(&right_switch)
            || !(1..=3).contains(&left_switch)
        {
            return None;
        }
        let signed = |i: usize| i16::from_le_bytes([b[i], b[i + 1]]);
        Some(Self {
            channels,
            right_switch,
            left_switch,
            mouse: [signed(6), signed(8), signed(10)],
            mouse_left: b[12] != 0,
            mouse_right: b[13] != 0,
            keys: u16::from_le_bytes([b[14], b[15]]),
        })
    }
}

/// Split DBUS frames at UART idle and publish only the latest one.
#[embassy_executor::task]
pub async fn remote_task(mut rx: RemoteRx) {
    // Twice a frame, so a read that fills the buffer shows a missing idle gap.
    let mut bytes = [0; 36];
    loop {
        match rx.read_until_idle(&mut bytes).await {
            Ok(n) => REMOTE.signal(Dr16::parse(&bytes[..n])),
            Err(_) => {
                REMOTE.signal(None);
                Timer::after(Duration::from_millis(1)).await;
            }
        }
    }
}

/// Program IWDG1 directly, since the control task owns the HAL watchdog. At the nominal
/// 32 kHz LSI, reload 799 is 100 ms and reload 0 is 125 us.
fn configure_panic_watchdog(reload: u16) {
    use pac::iwdg::vals::{Key, Pr};

    let wdg = pac::IWDG1;
    wdg.kr().write(|w| w.set_key(Key::START));
    wdg.kr().write(|w| w.set_key(Key::ENABLE));
    while wdg.sr().read().pvu() || wdg.sr().read().rvu() {
        core::hint::spin_loop();
    }
    wdg.pr().write(|w| w.set_pr(Pr::DIVIDE_BY4));
    wdg.rlr().write(|w| w.set_rl(reload));
    while wdg.sr().read().pvu() || wdg.sr().read().rvu() {
        core::hint::spin_loop();
    }
    wdg.kr().write(|w| w.set_key(Key::RESET));
}

/// Disable interrupts, give the probe up to 100 ms to read the message, then reset through
/// the watchdog. The next boot halts in `init` until a manual reset.
#[panic_handler]
fn panic(info: &PanicInfo<'_>) -> ! {
    static PANICKED: AtomicBool = AtomicBool::new(false);

    cortex_m::interrupt::disable();
    if !PANICKED.swap(true, Ordering::Relaxed) {
        configure_panic_watchdog(799);
        rtt_target::with_terminal_channel(|term| {
            term.set_mode(rtt_target::ChannelMode::BlockIfFull);
            let _ = writeln!(term.write(0), "{}", info);
            term.flush();
        });
    }
    configure_panic_watchdog(0);
    loop {
        core::hint::spin_loop();
    }
}
