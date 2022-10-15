#![no_std]
#![no_main]

use core::{cell::RefCell, ops::DerefMut};

use heapless::String;
use numtoa::NumToA;

use panic_halt as _;

use gd32vf103_pac as pac;
use gd32vf103xx_hal::{self as hal, prelude::*};
use hal::{delay::McycleDelay, eclic::{EclicExt, Level}, timer::Timer};
use embedded_hal::digital::v2::{InputPin, OutputPin};

use riscv::interrupt::Mutex;
// use ssd1306::{prelude::*, Builder, I2CDIBuilder};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

use embedded_graphics::{
    // image::{Image, ImageRaw},
    text::{Baseline, Text},
    mono_font::{MonoTextStyle, iso_8859_1::FONT_5X8},
    pixelcolor::BinaryColor,
    prelude::*,
};

type I2cInterfaceTypeAlias = I2CInterface<hal::i2c::BlockingI2c<pac::I2C0, (hal::gpio::gpiob::PB6<hal::gpio::Alternate<hal::gpio::OpenDrain>>, hal::gpio::gpiob::PB7<hal::gpio::Alternate<hal::gpio::OpenDrain>>)>>;
type DisplayTypeAlias = Ssd1306<I2cInterfaceTypeAlias, DisplaySize96x16, ssd1306::mode::BufferedGraphicsMode<DisplaySize96x16>>;

static G_DISP: Mutex<RefCell<Option<DisplayTypeAlias>>> = Mutex::new(RefCell::new(None));

static G_CURRENT_TIME: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));

static G_TIMER1: Mutex<RefCell<Option<Timer<pac::TIMER1>>>> = Mutex::new(RefCell::new(None));

// static G_RTC: Mutex<RefCell<Option<hal::rtc::Rtc>>> = Mutex::new(RefCell::new(None));

type BtnATypeAlias = hal::gpio::gpiob::PB1<hal::gpio::Input<hal::gpio::Floating>>;
static G_BTN_A: Mutex<RefCell<Option<BtnATypeAlias>>> = Mutex::new(RefCell::new(None));

#[riscv_rt::entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    // Use external 8MHz HXTAL and set PLL to get 96MHz system clock.
    let mut rcu = p
        .RCU
        .configure()
        .ext_hf_clock(8.mhz())
        // .sysclk(96.mhz())
        .sysclk(96.mhz())
        .freeze();
    let mut afio = p.AFIO.constrain(&mut rcu);

    let mut delay = McycleDelay::new(&rcu.clocks);

    let gpioa = p.GPIOA.split(&mut rcu);
    let gpiob = p.GPIOB.split(&mut rcu);

    // right button
    // Note that this pin is already pulled low externally via a 10K resistor
    // since it also operates the BOOT0 pin, so we don't need the internal
    // pull-down.
    let btn_a = gpiob.pb1.into_floating_input();

    riscv::interrupt::free(|cs| {
        *G_BTN_A.borrow(*cs).borrow_mut() = Some(btn_a);
    });

    // OLED reset: Pull low to reset.
    let mut oled_reset = gpioa
        .pa9
        .into_push_pull_output_with_state(hal::gpio::State::Low);

    let pb6_scl = gpiob.pb6.into_alternate_open_drain();
    let pb7_sda = gpiob.pb7.into_alternate_open_drain();

    // Set up i2c.
    let i2c0 = hal::i2c::BlockingI2c::i2c0(
        p.I2C0,
        (pb6_scl, pb7_sda),
        &mut afio,
        hal::i2c::Mode::Fast {
            frequency: 400_000.hz(),
            duty_cycle: hal::i2c::DutyCycle::Ratio2to1,
        },
        &mut rcu,
        1000,
        10,
        1000,
        1000,
    );   

    // OLED datasheet recommends 100 ms delay on power up.
    delay.delay_ms(100);

    // Init OLED.
    oled_reset.set_high().unwrap();

    // OLED datasheet recommends 3 us delay to wait for init.
    delay.delay_us(3);

    let interface = I2CDisplayInterface::new(i2c0);
    let mut disp = Ssd1306::new(interface, DisplaySize96x16, DisplayRotation::Rotate180)
        .into_buffered_graphics_mode();
    disp.init().unwrap();
    disp.clear();
    disp.flush().unwrap();

    riscv::interrupt::free(|cs| {
        *G_DISP.borrow(*cs).borrow_mut() = Some(disp);
    });

    pac::ECLIC::reset();
    pac::ECLIC::set_threshold_level(Level::L0);
    pac::ECLIC::set_level_priority_bits(hal::eclic::LevelPriorityBits::L3P1);

    // setup TIMER1 to fire an interrupt with 1 Hz frequency
    let mut timer1 = hal::timer::Timer::timer1(p.TIMER1, 1.hz(), &mut rcu);
    timer1.listen(hal::timer::Event::Update);

    riscv::interrupt::free(|cs| {
        *G_TIMER1.borrow(*cs).borrow_mut() = Some(timer1);
    });

    pac::ECLIC::setup(
        pac::Interrupt::TIMER1, 
        hal::eclic::TriggerType::Level, 
        hal::eclic::Level::L1, 
        hal::eclic::Priority::P1
    );

    unsafe { 
        pac::ECLIC::unmask(pac::Interrupt::TIMER1);
        riscv::interrupt::enable();
    };

    // couldn't get RTC working :(

    // let mut pmu = p.PMU;
    // let mut bkp = p.BKP.configure(&mut rcu, &mut pmu);
    // let rtc = hal::rtc::Rtc::rtc(p.RTC, &mut bkp);
    // riscv::interrupt::free(|cs| {
    //     *G_RTC.borrow(*cs).borrow_mut() = Some(rtc);
    // });

    loop {
        unsafe { riscv::asm::wfi(); }
    }
}

#[allow(non_snake_case)]
#[no_mangle]
fn TIMER1() {
    riscv::interrupt::free(|cs| {
        let current_time = *G_CURRENT_TIME.borrow(*cs).borrow_mut().deref_mut();

        if let Some(ref mut timer1) = *G_TIMER1.borrow(*cs).borrow_mut().deref_mut() {
            timer1.clear_update_interrupt_flag();
        }

        if let Some(ref mut btn_a) = *G_BTN_A.borrow(*cs).borrow_mut().deref_mut() {
            let character_style = MonoTextStyle::new(&FONT_5X8, BinaryColor::On);
            let mut buf = [0u8; 20];
            let mut text: String<50> = String::from("T: ");
            text.push_str(current_time.numtoa_str(10, &mut buf)).unwrap();

            if btn_a.is_high().unwrap() {
                text.push_str(", pressed").unwrap();
            } else {
                text.push_str(", not pressed").unwrap();
            }

            if let Some(ref mut disp) = *G_DISP.borrow(*cs).borrow_mut().deref_mut() {
                disp.clear();
                Text::with_baseline(
                    &text,
                    Point::new(0, 0),
                    character_style,
                    Baseline::Top,
                )
                .draw(disp).unwrap();
                disp.flush().unwrap();
            }
        }

        *G_CURRENT_TIME.borrow(*cs).borrow_mut().deref_mut() += 1;
    });
}
