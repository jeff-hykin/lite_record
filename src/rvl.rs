//! RVL, the depth codec from Wilson 2017, "Fast Lossless Depth Image Compression".
//!
//! Depth is 16-bit and every general-purpose image codec either truncates it or
//! costs far more than the 33 ms a 30 Hz frame gets. RVL is neither: it runs in
//! a couple of milliseconds on a Pi 5 and is bit-exact. Measured here on real
//! 1280x720 D435if frames, against the raw+lz4 this replaces — 15.2x -> 31.0x on
//! a sparse scene, 4.0x -> 5.4x on a dense one, both with zstd chunks.
//!
//! The bytes are laid out exactly as ROS 2's `compressed_depth_image_transport`
//! writes them, so Foxglove's RVL extension reads our recordings unmodified.

/// `compressionFormat::INV_DEPTH`, which is what the ROS encoder writes even for
/// 16UC1. The two quantisation parameters that follow are only meaningful for
/// 32FC1 inverse depth and are zero here.
const INV_DEPTH: i32 = 0;

/// int32 format + 2x float32 quantisation + int32 cols + int32 rows.
const HEADER_BYTES: usize = 20;

struct Writer {
    output: Vec<u8>,
    word: u32,
    nibbles_written: u32,
}

impl Writer {
    /// Values are packed as base-8 nibbles, low three bits of data and a high
    /// continuation bit, four to a little-endian word.
    fn encode(&mut self, mut value: u32) {
        loop {
            let mut nibble = value & 0x7;
            value >>= 3;
            if value != 0 {
                nibble |= 0x8;
            }
            self.word = (self.word << 4) | nibble;
            self.nibbles_written += 1;
            if self.nibbles_written == 8 {
                self.output.extend_from_slice(&self.word.to_le_bytes());
                self.nibbles_written = 0;
                self.word = 0;
            }
            if value == 0 {
                return;
            }
        }
    }
}

/// Encodes 16-bit depth as a `compressedDepth rvl` payload, header included.
pub fn compress(depth: &[u16], width: u32, height: u32) -> Vec<u8> {
    let mut output = Vec::with_capacity(HEADER_BYTES + depth.len());
    output.extend_from_slice(&INV_DEPTH.to_le_bytes());
    output.extend_from_slice(&0f32.to_le_bytes());
    output.extend_from_slice(&0f32.to_le_bytes());
    output.extend_from_slice(&(width as i32).to_le_bytes());
    output.extend_from_slice(&(height as i32).to_le_bytes());

    let mut writer = Writer { output, word: 0, nibbles_written: 0 };
    let mut previous: i32 = 0;
    let mut index = 0;
    while index < depth.len() {
        // Zero means "no return", and on a depth image those runs are long,
        // which is the whole reason this beats a general-purpose codec.
        let zeros_start = index;
        while index < depth.len() && depth[index] == 0 {
            index += 1;
        }
        writer.encode((index - zeros_start) as u32);
        let nonzeros_start = index;
        while index < depth.len() && depth[index] != 0 {
            index += 1;
        }
        writer.encode((index - nonzeros_start) as u32);
        for &current in &depth[nonzeros_start..index] {
            let delta = current as i32 - previous;
            writer.encode(((delta << 1) ^ (delta >> 31)) as u32);
            previous = current as i32;
        }
    }

    let Writer { mut output, word, nibbles_written } = writer;
    if nibbles_written != 0 {
        output.extend_from_slice(&(word << (4 * (8 - nibbles_written))).to_le_bytes());
    }
    output
}

struct Reader<'a> {
    input: &'a [u8],
    position: usize,
    word: u32,
    nibbles_left: u32,
}

impl Reader<'_> {
    fn decode(&mut self) -> Option<u32> {
        let mut value: u32 = 0;
        let mut bits = 29;
        loop {
            if self.nibbles_left == 0 {
                let word = self.input.get(self.position..self.position + 4)?;
                self.word = u32::from_le_bytes([word[0], word[1], word[2], word[3]]);
                self.position += 4;
                self.nibbles_left = 8;
            }
            let nibble = self.word & 0xf000_0000;
            value |= (nibble << 1) >> bits;
            self.word <<= 4;
            self.nibbles_left -= 1;
            bits -= 3;
            if nibble & 0x8000_0000 == 0 {
                return Some(value);
            }
        }
    }
}

/// Reverses [`compress`]. `None` on a truncated or malformed payload.
pub fn decompress(payload: &[u8]) -> Option<(Vec<u16>, u32, u32)> {
    let header = payload.get(..HEADER_BYTES)?;
    let width = i32::from_le_bytes(header[12..16].try_into().ok()?);
    let height = i32::from_le_bytes(header[16..20].try_into().ok()?);
    if width <= 0 || height <= 0 {
        return None;
    }
    let pixel_count = (width as usize).checked_mul(height as usize)?;

    let mut reader =
        Reader { input: &payload[HEADER_BYTES..], position: 0, word: 0, nibbles_left: 0 };
    let mut depth = Vec::with_capacity(pixel_count);
    let mut previous: i32 = 0;
    while depth.len() < pixel_count {
        let zeros = reader.decode()? as usize;
        if depth.len() + zeros > pixel_count {
            return None;
        }
        depth.resize(depth.len() + zeros, 0);
        let nonzeros = reader.decode()? as usize;
        if depth.len() + nonzeros > pixel_count {
            return None;
        }
        for _ in 0..nonzeros {
            let zigzag = reader.decode()? as i32;
            previous += (zigzag >> 1) ^ -(zigzag & 1);
            depth.push(previous as u16);
        }
    }
    Some((depth, width as u32, height as u32))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn round_trip(depth: &[u16], width: u32, height: u32) {
        let (decoded, decoded_width, decoded_height) =
            decompress(&compress(depth, width, height)).unwrap();
        assert_eq!(decoded, depth);
        assert_eq!((decoded_width, decoded_height), (width, height));
    }

    #[test]
    fn round_trips_a_depth_like_frame() {
        // Zero background with a couple of smooth blobs, which is what a depth
        // frame actually looks like: long zero runs and small deltas.
        let (width, height) = (64u32, 48u32);
        let mut depth = vec![0u16; (width * height) as usize];
        for row in 8..24 {
            for column in 10..40 {
                depth[(row * width + column) as usize] = 800 + row as u16 * 3 + column as u16;
            }
        }
        round_trip(&depth, width, height);
    }

    #[test]
    fn round_trips_the_extremes() {
        // 0xFFFF is a real value out of a RealSense (saturated), and it is the
        // worst case for the zigzag delta, so it must survive exactly.
        round_trip(&[0, 0xffff, 0, 0xffff, 1, 0xffff, 0, 0], 4, 2);
        round_trip(&vec![0u16; 256], 16, 16);
        round_trip(&(0..256).map(|value| value as u16).collect::<Vec<_>>(), 16, 16);
    }

    #[test]
    fn all_zero_frame_is_tiny() {
        // The sparse case is the one this codec exists for, so guard the win.
        let encoded = compress(&vec![0u16; 1280 * 720], 1280, 720);
        assert!(encoded.len() < HEADER_BYTES + 32, "{} bytes", encoded.len());
    }

    #[test]
    fn rejects_a_truncated_payload() {
        let encoded = compress(&[0, 1, 2, 3, 0, 0, 4, 5], 4, 2);
        assert!(decompress(&encoded[..HEADER_BYTES - 1]).is_none());
        assert!(decompress(&encoded[..HEADER_BYTES + 2]).is_none());
    }

    /// The point of matching ROS's byte layout is that other people's decoders
    /// read our recordings, so an independent one is the only test that proves
    /// it. re_rvl is Rerun's, and it is what Foxglove's RVL extension is built
    /// on. Its 16UC1 path widens u16 to f32, which is exact.
    #[test]
    fn an_independent_decoder_reads_our_bytes() {
        let (width, height) = (64u32, 48u32);
        let mut depth = vec![0u16; (width * height) as usize];
        for row in 5..40 {
            for column in 3..60 {
                depth[(row * width + column) as usize] = 500 + row as u16 * 7 + column as u16 * 2;
            }
        }
        depth[100] = 0xffff;

        let encoded = compress(&depth, width, height);
        let metadata = re_rvl::RosRvlMetadata::parse(&encoded).unwrap();
        assert_eq!((metadata.width, metadata.height), (width, height));
        assert!(!metadata.has_quantization());
        let decoded = re_rvl::decode_rvl_with_quantization(&encoded, &metadata).unwrap();
        assert_eq!(decoded, depth.iter().map(|&value| value as f32).collect::<Vec<_>>());
    }

    #[test]
    fn rejects_nonsense_dimensions() {
        let mut encoded = compress(&[0, 1], 2, 1);
        encoded[12..16].copy_from_slice(&(-1i32).to_le_bytes());
        assert!(decompress(&encoded).is_none());
    }
}
