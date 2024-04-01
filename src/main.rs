#![no_main]
#![no_std]

use hal::pwm;
use panic_halt as _;

use stm32f0xx_hal as hal;

use crate::hal::{
    gpio::*,
    pac::{interrupt, Interrupt, TIM17},
    prelude::*,
    time::Hertz,
    timers::*,
};

use cortex_m_rt::entry;
use stm32f0xx_hal::usb::{Peripheral, UsbBus};
use stm32f0xx_hal::pac;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};
use core::cell::RefCell;
use cortex_m::{interrupt::Mutex, peripheral::Peripherals as c_m_Peripherals};

// A type definition for the GPIO pin to be used for our LED
type LEDPIN = gpioa::PA1<Output<PushPull>>;

// Make LED pin globally available
static GLED: Mutex<RefCell<Option<LEDPIN>>> = Mutex::new(RefCell::new(None));

// Make timer interrupt registers globally available
static GINT: Mutex<RefCell<Option<Timer<TIM17>>>> = Mutex::new(RefCell::new(None));

// Define an interupt handler, i.e. function to call when interrupt occurs. Here if our external
// interrupt trips when the timer timed out
#[interrupt]
fn TIM17() {
    static mut LED: Option<LEDPIN> = None;
    static mut INT: Option<Timer<TIM17>> = None;

    let led = LED.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| {
            // Move LED pin here, leaving a None in its place
            GLED.borrow(cs).replace(None).unwrap()
        })
    });

    let int = INT.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| {
            // Move LED pin here, leaving a None in its place
            GINT.borrow(cs).replace(None).unwrap()
        })
    });

    led.toggle().ok();
    int.wait().ok();
}

#[entry]
fn main() -> ! {

    let mut dp = pac::Peripherals::take().unwrap();

    /*
     * IMPORTANT: if you have a chip in TSSOP20 (STM32F042F) or UFQFPN28 (STM32F042G) package,
     * and want to use USB, make sure you call `remap_pins(rcc, syscfg)`, otherwise the device will not enumerate.
     *
     * Uncomment the following function if the situation above applies to you.
     */

    stm32f0xx_hal::usb::remap_pins(&mut dp.RCC, &mut dp.SYSCFG);

    let mut rcc = dp
    .RCC
    .configure()
    .hsi48()
    .enable_crs(dp.CRS)
    .sysclk(48.mhz())
    .pclk(24.mhz())
    .freeze(&mut dp.FLASH);

    let gpioa = dp.GPIOA.split(&mut rcc);

    let usb = Peripheral {
        usb: dp.USB,
        pin_dm: gpioa.pa11,
        pin_dp: gpioa.pa12,
    };

    let usb_bus = UsbBus::new(usb);

    let mut serial = SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0xfeed, 0xbeef))
        .manufacturer("Cepeda's productions")
        .product("Turonet")
        .serial_number("0x1234")
        .device_class(USB_CLASS_CDC)
        .build();

    // Set up a timer expiring after 1s
    let mut timer = Timer::tim17(dp.TIM17, Hertz(1), &mut rcc);

    // Generate an interrupt when the timer expires
    timer.listen(Event::TimeOut);

    cortex_m::interrupt::free(move |cs| {
        // (Re-)configure PA8 as output
        let led1 = gpioa.pa8.into_alternate_af2(cs);
        // (Re-)configure PA1 as output
        let led2 = gpioa.pa1.into_push_pull_output(cs);
        let pwm = pwm::tim1(dp.TIM1, led1, &mut rcc, 1u32.hz());

        let mut ch1 = pwm;
        let max_duty = ch1.get_max_duty();
        ch1.set_duty(max_duty / 2);
        ch1.enable();
        
        // Move the pin into our global storage
        *GLED.borrow(cs).borrow_mut() = Some(led2);

        // Move the timer into our global storage
        *GINT.borrow(cs).borrow_mut() = Some(timer);
    });

    // Enable TIM7 IRQ, set prio 1 and clear any pending IRQs
    let mut nvic = c_m_Peripherals::take().unwrap().NVIC;
    unsafe {
        nvic.set_priority(Interrupt::TIM17, 1);
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM17);
    }
    cortex_m::peripheral::NVIC::unpend(Interrupt::TIM17);

    loop {
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }

        let mut buf = [0u8; 64];

        match serial.read(&mut buf) {
            Ok(count) if count > 0 => {
                // Echo back in upper case
                for c in buf[0..count].iter_mut() {
                    if 0x61 <= *c && *c <= 0x7a {
                        *c &= !0x20;
                    }
                }

                let mut write_offset = 0;
                while write_offset < count {
                    match serial.write(&buf[write_offset..count]) {
                        Ok(len) if len > 0 => {
                            write_offset += len;
                        }
                        _ => {}
                    }
                }
            }
            _ => {}
        }
    }
}
