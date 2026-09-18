//! LCM encoding for exactly one message: `sensor_msgs/PointCloud2`.
//!
//! Everything else this recorder writes is CDR, inside an mcap. A `.pc2.lcm` is
//! the odd one out -- a single LCM-encoded cloud in a bare file, which is what
//! dimos' `PointCloud2.lcm_encode()` produces and what `dimos map view` and the
//! rest of that tooling read.
//!
//! One message type does not justify depending on the generated `lcm-msgs`
//! crate, which would put a git dependency in the release build, so the wire
//! format is written out here. It is taken from dimos' own generated class
//! (`dimos_lcm/python_lcm_msgs/lcm_msgs/sensor_msgs/PointCloud2.py`), not
//! guessed:
//!
//! ```text
//! u64  fingerprint = 0xf5eb_3da1_c285_3175
//! i32  fields_length          <- hoisted
//! i32  data_length            <- hoisted
//! i32  header.seq
//! i32  header.stamp.sec
//! i32  header.stamp.nsec
//! str  header.frame_id
//! i32  height
//! i32  width
//! for each field:
//!     str  name
//!     i32  offset
//!     u8   datatype
//!     i32  count
//! i8   is_bigendian
//! i32  point_step
//! i32  row_step
//! u8[] data                   <- data_length bytes, no length prefix of its own
//! i8   is_dense
//! ```
//!
//! Two things about that shape are easy to get wrong, and both were learned
//! from real blobs rather than from the definition:
//!
//! - **The array size fields are hoisted to the front**, ahead of the header,
//!   in declaration order. So the frame does not sit at a fixed offset: a
//!   `PointCloud2` carries two such sizes and a `CompressedImage` one, which is
//!   why their frames land 28 and 24 bytes in respectively.
//! - **A string's length counts its own NUL terminator**, and the terminator is
//!   written. `"livox_frame"` encodes as length 12.
//!
//! Everything is big-endian and packed -- LCM does no alignment padding.

use crate::msgs::PointCloud2;

/// dimos' fingerprint for `sensor_msgs.PointCloud2`, which folds in the hashes
/// of `std_msgs.Header` and `PointField`. Read back from the generated class,
/// and matched against the first eight bytes of a cloud blob dimos wrote.
const POINT_CLOUD2_FINGERPRINT: u64 = 0xf5eb_3da1_c285_3175;

fn put_i32(out: &mut Vec<u8>, value: i32) {
    out.extend_from_slice(&value.to_be_bytes());
}

/// An LCM string: a big-endian length that *includes* the terminator, the
/// bytes, then the terminator.
fn put_string(out: &mut Vec<u8>, value: &str) {
    put_i32(out, value.len() as i32 + 1);
    out.extend_from_slice(value.as_bytes());
    out.push(0);
}

/// The bytes of a `.pc2.lcm`: one LCM-encoded `sensor_msgs/PointCloud2`,
/// byte-for-byte what dimos' `PointCloud2.lcm_encode()` would write.
pub fn point_cloud2(cloud: &PointCloud2) -> Vec<u8> {
    let mut out = Vec::with_capacity(cloud.data.len() + 256);
    out.extend_from_slice(&POINT_CLOUD2_FINGERPRINT.to_be_bytes());
    put_i32(&mut out, cloud.fields.len() as i32);
    put_i32(&mut out, cloud.data.len() as i32);
    // header: seq is not carried anywhere in this recorder, and dimos ignores it
    put_i32(&mut out, 0);
    put_i32(&mut out, cloud.header.stamp_sec);
    put_i32(&mut out, cloud.header.stamp_nsec);
    put_string(&mut out, &cloud.header.frame_id);
    put_i32(&mut out, cloud.height as i32);
    put_i32(&mut out, cloud.width as i32);
    for field in &cloud.fields {
        put_string(&mut out, &field.name);
        put_i32(&mut out, field.offset as i32);
        out.push(field.datatype);
        put_i32(&mut out, field.count as i32);
    }
    out.push(u8::from(cloud.is_bigendian));
    put_i32(&mut out, cloud.point_step as i32);
    put_i32(&mut out, cloud.row_step as i32);
    out.extend_from_slice(&cloud.data);
    out.push(u8::from(cloud.is_dense));
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::msgs::{Header, PointField};

    fn xyz_cloud(points: &[[f32; 3]], frame: &str) -> PointCloud2 {
        let mut data = Vec::with_capacity(points.len() * 12);
        for point in points {
            for value in point {
                data.extend_from_slice(&value.to_le_bytes());
            }
        }
        PointCloud2 {
            header: Header { stamp_sec: 7, stamp_nsec: 8, frame_id: frame.into() },
            height: 1,
            width: points.len() as u32,
            fields: vec![
                PointField { name: "x".into(), offset: 0, datatype: 7, count: 1 },
                PointField { name: "y".into(), offset: 4, datatype: 7, count: 1 },
                PointField { name: "z".into(), offset: 8, datatype: 7, count: 1 },
            ],
            is_bigendian: false,
            point_step: 12,
            row_step: 12 * points.len() as u32,
            data,
            is_dense: true,
        }
    }

    /// Spelled out byte by byte rather than round-tripped through this same
    /// code, which would agree with itself no matter what it wrote.
    #[test]
    fn the_leading_bytes_are_the_layout_dimos_generated() {
        let encoded = point_cloud2(&xyz_cloud(&[[1.0, 2.0, 3.0]], "odom"));
        let mut want = Vec::new();
        want.extend_from_slice(&0xf5eb_3da1_c285_3175u64.to_be_bytes()); // fingerprint
        want.extend_from_slice(&3i32.to_be_bytes()); // fields_length, hoisted
        want.extend_from_slice(&12i32.to_be_bytes()); // data_length, hoisted
        want.extend_from_slice(&0i32.to_be_bytes()); // seq
        want.extend_from_slice(&7i32.to_be_bytes()); // stamp.sec
        want.extend_from_slice(&8i32.to_be_bytes()); // stamp.nsec
        want.extend_from_slice(&5i32.to_be_bytes()); // "odom" + NUL
        want.extend_from_slice(b"odom\0");
        want.extend_from_slice(&1i32.to_be_bytes()); // height
        want.extend_from_slice(&1i32.to_be_bytes()); // width
        assert_eq!(&encoded[..want.len()], &want[..]);
    }

    /// The trailer is as easy to get wrong as the header: the payload carries no
    /// length prefix of its own, and `is_dense` sits after it.
    #[test]
    fn the_payload_is_followed_only_by_is_dense() {
        let cloud = xyz_cloud(&[[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], "odom");
        let encoded = point_cloud2(&cloud);
        assert_eq!(*encoded.last().unwrap(), 1, "is_dense");
        let payload_end = encoded.len() - 1;
        assert_eq!(&encoded[payload_end - cloud.data.len()..payload_end], &cloud.data[..]);
    }

    /// A frame's length counts the terminator it is followed by. Getting this
    /// off by one shifts everything after it.
    #[test]
    fn a_frame_id_counts_its_own_terminator() {
        let encoded = point_cloud2(&xyz_cloud(&[], "livox_frame"));
        let at = 8 + 4 * 5; // fingerprint, two sizes, seq, sec, nsec
        assert_eq!(i32::from_be_bytes(encoded[at..at + 4].try_into().unwrap()), 12);
        assert_eq!(&encoded[at + 4..at + 16], b"livox_frame\0");
    }

    /// Each PointField is a string then three numbers, one of which is a single
    /// byte -- and LCM packs, so nothing is padded back to four.
    #[test]
    fn fields_are_packed_with_a_one_byte_datatype() {
        let encoded = point_cloud2(&xyz_cloud(&[], "odom"));
        let at = 8 + 4 * 5 + 4 + 5 + 4 + 4; // through the header, height and width
        assert_eq!(i32::from_be_bytes(encoded[at..at + 4].try_into().unwrap()), 2); // "x" + NUL
        assert_eq!(&encoded[at + 4..at + 6], b"x\0");
        assert_eq!(i32::from_be_bytes(encoded[at + 6..at + 10].try_into().unwrap()), 0); // offset
        assert_eq!(encoded[at + 10], 7); // datatype, one byte
        assert_eq!(i32::from_be_bytes(encoded[at + 11..at + 15].try_into().unwrap()), 1); // count
        // the next field starts immediately, with no padding
        assert_eq!(i32::from_be_bytes(encoded[at + 15..at + 19].try_into().unwrap()), 2);
        assert_eq!(&encoded[at + 19..at + 21], b"y\0");
    }
}
