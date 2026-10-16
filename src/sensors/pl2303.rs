//! A Prolific PL2303 USB-serial bridge driven from userspace.
//!
//! macOS has no driver for the newer (HXN family) PL2303 chips, which is what
//! the common GPS pucks ship with, so no `/dev/cu.*` node ever appears. The
//! protocol is small enough to speak directly: reset the pipes, set the line
//! coding, raise DTR/RTS, then read the bulk-in endpoint. The requests follow
//! Linux's `drivers/usb/serial/pl2303.c`.

use std::io::Read;
use std::time::Duration;

use anyhow::{bail, Context, Result};
use nusb::transfer::{Bulk, ControlIn, ControlOut, ControlType, Direction, In, Recipient};
use nusb::descriptors::TransferType;
use nusb::{DeviceInfo, MaybeFuture};

pub const VENDOR_ID: u16 = 0x067b;
/// The HXN family (PL2303GC/GB/GT/GL/GE/GS), the only ones handled here.
const HXN_PRODUCT_IDS: [u16; 6] = [0x23a3, 0x23b3, 0x23c3, 0x23d3, 0x23e3, 0x23f3];

const VENDOR_READ_REQUEST: u8 = 0x81;
const VENDOR_WRITE_REQUEST: u8 = 0x80;
const RESET_REGISTER: u16 = 0x07;
const RESET_BOTH_PIPES: u8 = 0x03;
const SET_LINE_REQUEST: u8 = 0x20;
const SET_CONTROL_REQUEST: u8 = 0x22;
const DTR_AND_RTS: u16 = 0x03;

const CONTROL_TIMEOUT: Duration = Duration::from_millis(500);
/// Short enough that the reader notices a stop request promptly.
const READ_TIMEOUT: Duration = Duration::from_millis(200);

/// The first attached HXN PL2303, if any.
pub fn find() -> Option<DeviceInfo> {
    nusb::list_devices()
        .wait()
        .ok()?
        .find(|device| device.vendor_id() == VENDOR_ID && HXN_PRODUCT_IDS.contains(&device.product_id()))
}

pub fn attached(vendor: u16, product: u16) -> bool {
    nusb::list_devices()
        .wait()
        .map(|mut devices| devices.any(|device| device.vendor_id() == vendor && device.product_id() == product))
        .unwrap_or(false)
}

/// Opens the bridge at `baud`, 8N1, returning a name for it and its byte stream.
pub fn open(info: &DeviceInfo, baud: u32) -> Result<(String, impl Read + Send)> {
    let name = format!("usb {:04x}:{:04x} (pl2303)", info.vendor_id(), info.product_id());
    let device = info.open().wait().with_context(|| format!("opening {name}"))?;
    let interface = device
        .detach_and_claim_interface(0)
        .wait()
        .with_context(|| format!("claiming {name}"))?;

    let vendor_out = |value: u16, index: u16| ControlOut {
        control_type: ControlType::Vendor,
        recipient: Recipient::Device,
        request: VENDOR_WRITE_REQUEST,
        value,
        index,
        data: &[],
    };
    let reset = interface
        .control_in(
            ControlIn {
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
                request: VENDOR_READ_REQUEST,
                value: RESET_REGISTER,
                index: 0,
                length: 1,
            },
            CONTROL_TIMEOUT,
        )
        .wait()
        .context("reading the pl2303 reset register")?;
    let current = reset.first().copied().unwrap_or(0);
    interface
        .control_out(vendor_out(RESET_REGISTER, (current | RESET_BOTH_PIPES) as u16), CONTROL_TIMEOUT)
        .wait()
        .context("resetting the pl2303 pipes")?;

    let mut line_coding = [0u8; 7];
    line_coding[..4].copy_from_slice(&baud.to_le_bytes());
    // Then one stop bit, no parity, eight data bits.
    line_coding[6] = 8;
    interface
        .control_out(
            ControlOut {
                control_type: ControlType::Class,
                recipient: Recipient::Interface,
                request: SET_LINE_REQUEST,
                value: 0,
                index: 0,
                data: &line_coding,
            },
            CONTROL_TIMEOUT,
        )
        .wait()
        .with_context(|| format!("setting the pl2303 to {baud} baud"))?;
    interface
        .control_out(
            ControlOut {
                control_type: ControlType::Class,
                recipient: Recipient::Interface,
                request: SET_CONTROL_REQUEST,
                value: DTR_AND_RTS,
                index: 0,
                data: &[],
            },
            CONTROL_TIMEOUT,
        )
        .wait()
        .context("raising the pl2303's DTR/RTS")?;

    let bulk_in = interface
        .descriptor()
        .and_then(|descriptor| {
            descriptor
                .endpoints()
                .find(|endpoint| endpoint.transfer_type() == TransferType::Bulk && endpoint.direction() == Direction::In)
                .map(|endpoint| endpoint.address())
        });
    let Some(address) = bulk_in else {
        bail!("{name} has no bulk-in endpoint");
    };
    let reader = interface
        .endpoint::<Bulk, In>(address)
        .with_context(|| format!("opening {name}'s bulk-in endpoint"))?
        .reader(256)
        .with_num_transfers(4)
        .with_read_timeout(READ_TIMEOUT);
    Ok((name, reader))
}
