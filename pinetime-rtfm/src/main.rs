#![no_main]
#![no_std]
#![feature(alloc_error_handler)]

#[macro_use]
extern crate alloc;

#[allow(unused_imports)]
use panic_semihosting;

use alloc::vec::Vec;
use alloc_cortex_m::CortexMHeap;
use core::alloc::Layout;
use core::time::Duration;
use cortex_m::asm;
use embedded_graphics::prelude::*;
use embedded_graphics::{
    fonts::{Font12x16, Text},
    image::{Image, ImageRawLE},
    pixelcolor::Rgb565,
    primitives::rectangle::Rectangle,
    style::{PrimitiveStyleBuilder, TextStyleBuilder},
};
use lvgl::{Align, Button, Color, DisplayDriver, Label, Object, UI};
use nrf52832_hal::gpio::{p0, Level, Output, PushPull};
use nrf52832_hal::prelude::*;
use nrf52832_hal::{self as hal, pac};
use numtoa::NumToA;
use rtfm::app;
use rtfm::cyccnt::U32Ext;
use rtt_target::{rprintln, rtt_init_print};
use st7789::{self, Orientation};

mod delay;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

const HEAP_SIZE: usize = 1024; // in bytes

const LCD_W: u16 = 240;
const LCD_H: u16 = 240;

const FERRIS_W: u16 = 86;
const FERRIS_H: u16 = 64;
const FERRIS_Y_OFFSET: u16 = 80;

const MARGIN: u16 = 10;

const BACKGROUND_COLOR: Rgb565 = Rgb565::new(200, 0, 150);

const CLOCK_FREQUENCY: u32 = 64_000_000;

#[app(device = nrf52832_hal::pac, peripherals = true, monotonic = rtfm::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        // LittlevGL Graphical Interface
        ui: lvgl::UI,
    }

    #[init()]
    fn init(cx: init::Context) -> init::LateResources {
        let _p = cx.core;
        let dp = cx.device;

        // Init RTT
        rtt_init_print!();
        rprintln!("Initializing…");

        // Set up clocks
        let _clocks = hal::clocks::Clocks::new(dp.CLOCK);

        // Set up delay timer
        let delay = delay::TimerDelay::new(dp.TIMER0);

        // Set up GPIO peripheral
        let gpio = hal::gpio::p0::Parts::new(dp.P0);

        // Enable backlight
        let _backlight_low = gpio.p0_14.into_push_pull_output(Level::High);
        let _backlight_mid = gpio.p0_22.into_push_pull_output(Level::High);
        let mut backlight_high = gpio.p0_23.into_push_pull_output(Level::High);
        backlight_high.set_low().unwrap();

        // Set up SPI pins
        let spi_clk = gpio.p0_02.into_push_pull_output(Level::Low).degrade();
        let spi_mosi = gpio.p0_03.into_push_pull_output(Level::Low).degrade();
        let spi_miso = gpio.p0_04.into_floating_input().degrade();
        let spi_pins = hal::spim::Pins {
            sck: spi_clk,
            miso: Some(spi_miso),
            mosi: Some(spi_mosi),
        };

        // Set up LCD pins
        // LCD_CS (P0.25): Chip select
        let mut lcd_cs = gpio.p0_25.into_push_pull_output(Level::Low);
        // LCD_RS (P0.18): Data/clock pin
        let lcd_dc = gpio.p0_18.into_push_pull_output(Level::Low);
        // LCD_RESET (P0.26): Display reset
        let lcd_rst = gpio.p0_26.into_push_pull_output(Level::Low);

        // Initialize SPI
        let spi = hal::Spim::new(
            dp.SPIM1,
            spi_pins,
            // Use SPI at 8MHz (the fastest clock available on the nRF52832)
            // because otherwise refreshing will be super slow.
            hal::spim::Frequency::M8,
            // SPI must be used in mode 3. Mode 0 (the default) won't work.
            hal::spim::MODE_3,
            0,
        );

        // Chip select must be held low while driving the display. It must be high
        // when using other SPI devices on the same bus (such as external flash
        // storage) so that the display controller won't respond to the wrong
        // commands.
        lcd_cs.set_low().unwrap();

        // Initialize the allocator
        unsafe { ALLOCATOR.init(cortex_m_rt::heap_start() as usize, HEAP_SIZE) }

        // Initialize LCD
        let mut lcd = st7789::ST7789::new(spi, lcd_dc, lcd_rst, LCD_W, LCD_H, delay);
        lcd.init().unwrap();
        lcd.set_orientation(&Orientation::Portrait).unwrap();

        // Draw something onto the LCD
        let backdrop_style = PrimitiveStyleBuilder::new()
            .fill_color(BACKGROUND_COLOR)
            .build();
        Rectangle::new(Point::new(0, 0), Point::new(LCD_W as i32, LCD_H as i32))
            .into_styled(backdrop_style)
            .draw(&mut lcd)
            .unwrap();

        let text_style = TextStyleBuilder::new(Font12x16)
            .text_color(Rgb565::WHITE)
            .background_color(BACKGROUND_COLOR);

        let mut ui = UI::init().unwrap();

        let display_driver = DisplayDriver::new(&mut lcd);
        ui.disp_drv_register(display_driver);

        let mut screen = ui.scr_act();
        // Set black background screen
        // let mut screen_style = lvgl::Style::new();
        // screen_style.set_body_main_color(lvgl::Color::from_rgb((0, 0, 0)));
        // screen_style.set_body_grad_color(lvgl::Color::from_rgb((0, 0, 0)));
        // screen.set_style(screen_style);

        let mut label = Label::new(&mut screen);
        let mut top_lbl_style = lvgl::Style::new();
        top_lbl_style.set_text_color(Color::from_rgb((200, 0, 0)));
        label.set_style(top_lbl_style);
        label.set_text("It works!!1");
        label.set_align(&mut screen, Align::InTopMid, 0, 0);

        let mut button = Button::new(&mut screen);
        button.set_size(220, 80);

        let mut label = Label::new(&mut button);
        label.set_text("Open");
        button.set_align(&mut screen, lvgl::Align::Center, 0, 0);

        for _ in 0..100 {
            ui.tick_inc(Duration::from_millis(5));
            ui.task_handler();
        }
        //
        // Text::new("Updated!", Point::new(10, 50))
        //     .into_styled(text_style.build())
        //     .draw(&mut lcd)
        //     .unwrap();

        init::LateResources { ui }
    }

    // Provide unused interrupts to RTFM for its scheduling
    extern "C" {
        fn SWI0_EGU0();
        fn SWI1_EGU1();
        fn SWI2_EGU2();
        fn SWI3_EGU3();
        fn SWI4_EGU4();
        fn SWI5_EGU5();
    }
};

#[alloc_error_handler]
fn alloc_error(_layout: Layout) -> ! {
    asm::bkpt();

    rprintln!("OOM x(");
    loop {}
}
