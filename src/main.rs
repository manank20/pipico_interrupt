#![no_std]
#![no_main]

use cortex_m::asm::delay;
use cortex_m_rt::entry;

use embedded_hal::digital::v2::OutputPin;

use embedded_hal::digital::v2::ToggleableOutputPin;
use rp_pico::hal::gpio::Interrupt::EdgeLow;
use rp_pico::hal::pac::interrupt;
use rp_pico::hal::gpio;

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use embedded_time::rate::*;

use panic_halt as _;

use rp_pico::hal::prelude::*;

use rp_pico::hal::pac;

use rp_pico::hal;



static GLOBAL_PINS: Mutex<RefCell<Option<(gpio::Pin<gpio::bank0::Gpio25, gpio::PushPullOutput>,gpio::Pin<gpio::bank0::Gpio12, gpio::PullDownInput>)>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let sio = hal::Sio::new(pac.SIO);

    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();
    let switch = pins.gpio12.into_pull_down_input();

    switch.set_interrupt_enabled(EdgeLow, true);
    cortex_m::interrupt::free(|cs| {
            GLOBAL_PINS.borrow(cs).replace(Some((led_pin,switch)));
    });


    unsafe {
        pac::NVIC::unmask(pac::interrupt::IO_IRQ_BANK0);
    }

    loop {
        delay.delay_ms(900_000_000);
}
}

#[interrupt]
fn IO_IRQ_BANK0() {
    static mut LED_AND_BUTTON: Option<(gpio::Pin<gpio::bank0::Gpio25, gpio::PushPullOutput>,gpio::Pin<gpio::bank0::Gpio12, gpio::PullDownInput>)> = None;
    if LED_AND_BUTTON.is_none(){
        cortex_m::interrupt::free(|cs| {
            *LED_AND_BUTTON = GLOBAL_PINS.borrow(cs).take();
        });
    };

    if let Some(gpios) = LED_AND_BUTTON {
        // borrow led and button by *destructuring* the tuple
        // these will be of type `&mut LedPin` and `&mut ButtonPin`, so we don't have
        // to move them back into the static after we use them
        let (led, button) = gpios;

        // toggle can't fail, but the embedded-hal traits always allow for it
        // we can discard the return value by assigning it to an unnamed variable
        let _ = led.toggle();

        // Our interrupt doesn't clear itself.
        // Do that now so we don't immediately jump back to this interrupt handler.
        button.clear_interrupt(EdgeLow);
    }
}
