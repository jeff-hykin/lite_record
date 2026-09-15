//! ROS2 CDR encoding. Every message we record goes through here, so the mcap
//! opens directly in Foxglove or `ros2 bag` without a translation step.

use crate::msgs::{
    CameraInfo, CompressedImage, Header, Imu, Odometry, PointCloud2, RawImage, TransformStamped,
};

/// Concatenated ros2msg text, which is what `message_encoding: "cdr"` readers
/// expect. Note ROS2 headers have no `seq` field even though the ROS1 ones do.
const TIME_MSG: &str = "\
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
";

const HEADER_MSG: &str = "\
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
";

const VECTOR3_MSG: &str = "\
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
";

const QUATERNION_MSG: &str = "\
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
";

/// Clone so one encoding can be offered on more than one topic -- the same
/// intrinsics go out under both the canonical and the sibling `camera_info` name,
/// and encoding twice to avoid a copy would be the more expensive of the two.
#[derive(Clone)]
pub struct Encoded {
    pub schema_name: &'static str,
    pub schema_text: String,
    pub data: Vec<u8>,
}

pub fn raw_image(image: &RawImage) -> Encoded {
    let mut writer = CdrWriter::with_capacity(image.data.len() + 128);
    write_header(&mut writer, &image.header);
    writer.u32(image.height as u32);
    writer.u32(image.width as u32);
    writer.string(&image.encoding);
    writer.u8(image.is_bigendian);
    writer.u32(image.step as u32);
    writer.bytes(&image.data);
    Encoded {
        schema_name: "sensor_msgs/msg/Image",
        schema_text: format!(
            "std_msgs/Header header\n\
             uint32 height\n\
             uint32 width\n\
             string encoding\n\
             uint8 is_bigendian\n\
             uint32 step\n\
             uint8[] data\n\n{HEADER_MSG}\n{TIME_MSG}"
        ),
        data: writer.finish(),
    }
}

pub fn compressed_image(image: &CompressedImage) -> Encoded {
    let mut writer = CdrWriter::with_capacity(image.data.len() + 128);
    write_header(&mut writer, &image.header);
    writer.string(&image.format);
    writer.bytes(&image.data);
    Encoded {
        schema_name: "sensor_msgs/msg/CompressedImage",
        schema_text: format!(
            "std_msgs/Header header\n\
             string format\n\
             uint8[] data\n\n{HEADER_MSG}\n{TIME_MSG}"
        ),
        data: writer.finish(),
    }
}

pub fn tf_message(transforms: &[TransformStamped]) -> Encoded {
    let mut writer = CdrWriter::with_capacity(transforms.len() * 128 + 64);
    writer.u32(transforms.len() as u32);
    for transform in transforms {
        write_header(&mut writer, &transform.header);
        writer.string(&transform.child_frame_id);
        writer.f64_array(&transform.translation);
        writer.f64_array(&transform.rotation);
    }
    Encoded {
        schema_name: "tf2_msgs/msg/TFMessage",
        schema_text: format!(
            "geometry_msgs/TransformStamped[] transforms\n\n\
             ================================================================================\n\
             MSG: geometry_msgs/TransformStamped\n\
             std_msgs/Header header\n\
             string child_frame_id\n\
             geometry_msgs/Transform transform\n\n\
             ================================================================================\n\
             MSG: geometry_msgs/Transform\n\
             geometry_msgs/Vector3 translation\n\
             geometry_msgs/Quaternion rotation\n\
             {VECTOR3_MSG}{QUATERNION_MSG}{HEADER_MSG}\n{TIME_MSG}"
        ),
        data: writer.finish(),
    }
}

pub fn point_cloud2(cloud: &PointCloud2) -> Encoded {
    let mut writer = CdrWriter::with_capacity(cloud.data.len() + 256);
    write_header(&mut writer, &cloud.header);
    writer.u32(cloud.height);
    writer.u32(cloud.width);
    writer.u32(cloud.fields.len() as u32);
    for field in &cloud.fields {
        writer.string(&field.name);
        writer.u32(field.offset);
        writer.u8(field.datatype);
        writer.u32(field.count);
    }
    writer.boolean(cloud.is_bigendian);
    writer.u32(cloud.point_step);
    writer.u32(cloud.row_step);
    writer.bytes(&cloud.data);
    writer.boolean(cloud.is_dense);
    Encoded {
        schema_name: "sensor_msgs/msg/PointCloud2",
        schema_text: format!(
            "std_msgs/Header header\n\
             uint32 height\n\
             uint32 width\n\
             sensor_msgs/PointField[] fields\n\
             bool is_bigendian\n\
             uint32 point_step\n\
             uint32 row_step\n\
             uint8[] data\n\
             bool is_dense\n\n\
             ================================================================================\n\
             MSG: sensor_msgs/PointField\n\
             string name\n\
             uint32 offset\n\
             uint8 datatype\n\
             uint32 count\n{HEADER_MSG}\n{TIME_MSG}"
        ),
        data: writer.finish(),
    }
}

pub fn imu(imu: &Imu) -> Encoded {
    let mut writer = CdrWriter::with_capacity(384);
    write_header(&mut writer, &imu.header);
    writer.f64_array(&imu.orientation);
    writer.f64_array(&imu.orientation_covariance);
    writer.f64_array(&imu.angular_velocity);
    writer.f64_array(&imu.angular_velocity_covariance);
    writer.f64_array(&imu.linear_acceleration);
    writer.f64_array(&imu.linear_acceleration_covariance);
    Encoded {
        schema_name: "sensor_msgs/msg/Imu",
        schema_text: format!(
            "std_msgs/Header header\n\
             geometry_msgs/Quaternion orientation\n\
             float64[9] orientation_covariance\n\
             geometry_msgs/Vector3 angular_velocity\n\
             float64[9] angular_velocity_covariance\n\
             geometry_msgs/Vector3 linear_acceleration\n\
             float64[9] linear_acceleration_covariance\n\
             {QUATERNION_MSG}{VECTOR3_MSG}{HEADER_MSG}\n{TIME_MSG}"
        ),
        data: writer.finish(),
    }
}

pub fn camera_info(info: &CameraInfo) -> Encoded {
    let mut writer = CdrWriter::with_capacity(512);
    write_header(&mut writer, &info.header);
    writer.u32(info.height);
    writer.u32(info.width);
    writer.string(&info.distortion_model);
    writer.u32(info.distortion.len() as u32);
    writer.f64_array(&info.distortion);
    writer.f64_array(&info.intrinsics);
    writer.f64_array(&info.rectification);
    writer.f64_array(&info.projection);
    writer.u32(info.binning_x);
    writer.u32(info.binning_y);
    writer.u32(info.roi.x_offset);
    writer.u32(info.roi.y_offset);
    writer.u32(info.roi.height);
    writer.u32(info.roi.width);
    writer.boolean(info.roi.do_rectify);
    Encoded {
        schema_name: "sensor_msgs/msg/CameraInfo",
        schema_text: format!(
            "std_msgs/Header header\n\
             uint32 height\n\
             uint32 width\n\
             string distortion_model\n\
             float64[] d\n\
             float64[9] k\n\
             float64[9] r\n\
             float64[12] p\n\
             uint32 binning_x\n\
             uint32 binning_y\n\
             sensor_msgs/RegionOfInterest roi\n\n\
             ================================================================================\n\
             MSG: sensor_msgs/RegionOfInterest\n\
             uint32 x_offset\n\
             uint32 y_offset\n\
             uint32 height\n\
             uint32 width\n\
             bool do_rectify\n{HEADER_MSG}\n{TIME_MSG}"
        ),
        data: writer.finish(),
    }
}

fn write_header(writer: &mut CdrWriter, header: &Header) {
    writer.i32(header.stamp_sec);
    writer.u32(header.stamp_nsec as u32);
    writer.string(&header.frame_id);
}

pub struct CdrWriter {
    buffer: Vec<u8>,
}

impl CdrWriter {
    pub fn with_capacity(capacity: usize) -> Self {
        let mut buffer = Vec::with_capacity(capacity + 4);
        // Encapsulation header: little-endian CDR, no options.
        buffer.extend_from_slice(&[0x00, 0x01, 0x00, 0x00]);
        CdrWriter { buffer }
    }

    /// CDR alignment is measured from the start of the body, not the file, so
    /// the four encapsulation bytes do not count.
    fn align(&mut self, width: usize) {
        let body = self.buffer.len() - 4;
        let padding = (width - (body % width)) % width;
        self.buffer.resize(self.buffer.len() + padding, 0);
    }

    fn u8(&mut self, value: u8) {
        self.buffer.push(value);
    }

    fn u32(&mut self, value: u32) {
        self.align(4);
        self.buffer.extend_from_slice(&value.to_le_bytes());
    }

    fn i32(&mut self, value: i32) {
        self.align(4);
        self.buffer.extend_from_slice(&value.to_le_bytes());
    }

    fn f64(&mut self, value: f64) {
        self.align(8);
        self.buffer.extend_from_slice(&value.to_le_bytes());
    }

    fn boolean(&mut self, value: bool) {
        self.u8(value as u8);
    }

    /// No length prefix: in CDR a fixed-size array is just its elements, and a
    /// variable-length one gets its count written separately by the caller.
    fn f64_array(&mut self, values: &[f64]) {
        for value in values {
            self.f64(*value);
        }
    }

    fn string(&mut self, value: &str) {
        self.u32(value.len() as u32 + 1);
        self.buffer.extend_from_slice(value.as_bytes());
        self.buffer.push(0);
    }

    fn bytes(&mut self, value: &[u8]) {
        self.u32(value.len() as u32);
        self.buffer.extend_from_slice(value);
    }

    fn finish(self) -> Vec<u8> {
        self.buffer
    }
}

/// Mirror of `CdrWriter`. The tests use it to prove a message survives the round
/// trip; `crate::convert` uses it to read a recording back.
pub struct CdrReader<'a> {
    data: &'a [u8],
    offset: usize,
}

impl<'a> CdrReader<'a> {
    pub fn new(data: &'a [u8]) -> Self {
        assert_eq!(&data[..4], &[0x00, 0x01, 0x00, 0x00], "not little-endian CDR");
        CdrReader { data, offset: 4 }
    }

    fn align(&mut self, width: usize) {
        let body = self.offset - 4;
        self.offset += (width - (body % width)) % width;
    }

    pub fn u8(&mut self) -> u8 {
        let value = self.data[self.offset];
        self.offset += 1;
        value
    }

    pub fn boolean(&mut self) -> bool {
        self.u8() != 0
    }

    pub fn u32(&mut self) -> u32 {
        self.align(4);
        let value = u32::from_le_bytes(self.data[self.offset..self.offset + 4].try_into().unwrap());
        self.offset += 4;
        value
    }

    pub fn i32(&mut self) -> i32 {
        self.u32() as i32
    }

    pub fn f64(&mut self) -> f64 {
        self.align(8);
        let value = f64::from_le_bytes(self.data[self.offset..self.offset + 8].try_into().unwrap());
        self.offset += 8;
        value
    }

    pub fn f64_array<const N: usize>(&mut self) -> [f64; N] {
        let mut values = [0.0; N];
        for value in values.iter_mut() {
            *value = self.f64();
        }
        values
    }

    pub fn string(&mut self) -> String {
        let length = self.u32() as usize;
        let bytes = &self.data[self.offset..self.offset + length - 1];
        self.offset += length;
        String::from_utf8(bytes.to_vec()).unwrap()
    }

    pub fn bytes(&mut self) -> Vec<u8> {
        let length = self.u32() as usize;
        let bytes = self.data[self.offset..self.offset + length].to_vec();
        self.offset += length;
        bytes
    }

    pub fn header(&mut self) -> Header {
        Header {
            stamp_sec: self.i32(),
            stamp_nsec: self.u32() as i32,
            frame_id: self.string(),
        }
    }

    pub fn remaining(&self) -> usize {
        self.data.len().saturating_sub(self.offset)
    }

    /// `string` panics past the end of the buffer, which is fine for a message
    /// this program encoded itself and not fine for one read back off a card
    /// that filled up mid-write. These `try_` readers stop instead.
    pub fn try_string(&mut self) -> Option<String> {
        let start = self.offset;
        self.align(4);
        if self.remaining() < 4 {
            self.offset = start;
            return None;
        }
        let length = self.u32() as usize;
        if length == 0 || self.remaining() < length {
            self.offset = start;
            return None;
        }
        let text = String::from_utf8(self.data[self.offset..self.offset + length - 1].to_vec());
        self.offset += length;
        text.ok()
    }

    pub fn try_header(&mut self) -> Option<Header> {
        self.align(4);
        if self.remaining() < 12 {
            return None;
        }
        Some(Header {
            stamp_sec: self.i32(),
            stamp_nsec: self.u32() as i32,
            frame_id: self.try_string()?,
        })
    }

    pub fn try_f64_array<const N: usize>(&mut self) -> Option<[f64; N]> {
        self.align(8);
        if self.remaining() < 8 * N {
            return None;
        }
        Some(self.f64_array())
    }

    pub fn at_end(&self) -> bool {
        self.offset == self.data.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::msgs::{DistortionModel, PointField, RegionOfInterest, POINT_FIELD_FLOAT32};

    #[test]
    fn strings_pad_the_next_field_back_to_alignment() {
        let mut writer = CdrWriter::with_capacity(0);
        writer.string("ab");
        // 4 length bytes + "ab\0" leaves the body at 7.
        assert_eq!(writer.buffer.len() - 4, 7);
        writer.u32(1);
        assert_eq!(writer.buffer.len() - 4, 12);
    }

    #[test]
    fn camera_info_reads_back_field_for_field() {
        let info = CameraInfo {
            header: Header::new(12_000_000_400, "camera_color_optical_frame"),
            height: 720,
            width: 1280,
            distortion_model: DistortionModel::PlumbBob.as_str().to_string(),
            distortion: vec![0.1, -0.2, 0.001, 0.002, 0.05],
            intrinsics: [911.66, 0.0, 658.0, 0.0, 911.49, 368.99, 0.0, 0.0, 1.0],
            rectification: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            projection: [
                911.66, 0.0, 658.0, 0.0, 0.0, 911.49, 368.99, 0.0, 0.0, 0.0, 1.0, 0.0,
            ],
            binning_x: 0,
            binning_y: 0,
            roi: RegionOfInterest::default(),
        };
        let encoded = camera_info(&info);
        assert_eq!(encoded.schema_name, "sensor_msgs/msg/CameraInfo");

        let mut reader = CdrReader::new(&encoded.data);
        assert_eq!(reader.header(), info.header);
        assert_eq!(reader.u32(), 720);
        assert_eq!(reader.u32(), 1280);
        assert_eq!(reader.string(), "plumb_bob");
        assert_eq!(reader.u32(), 5);
        assert_eq!(reader.f64_array::<5>().to_vec(), info.distortion);
        assert_eq!(reader.f64_array::<9>(), info.intrinsics);
        assert_eq!(reader.f64_array::<9>(), info.rectification);
        assert_eq!(reader.f64_array::<12>(), info.projection);
        assert_eq!(reader.u32(), 0);
        assert_eq!(reader.u32(), 0);
        for _ in 0..4 {
            assert_eq!(reader.u32(), 0);
        }
        assert!(!reader.boolean());
        assert!(reader.at_end());
    }

    #[test]
    fn point_cloud2_reads_back_including_its_payload() {
        let points: Vec<u8> = (0..64u8).collect();
        let cloud = PointCloud2 {
            header: Header::new(5_000_000_000, "livox_frame"),
            height: 1,
            width: 4,
            fields: vec![
                PointField {
                    name: "x".into(),
                    offset: 0,
                    datatype: POINT_FIELD_FLOAT32,
                    count: 1,
                },
                PointField {
                    name: "y".into(),
                    offset: 4,
                    datatype: POINT_FIELD_FLOAT32,
                    count: 1,
                },
            ],
            is_bigendian: false,
            point_step: 16,
            row_step: 64,
            data: points.clone(),
            is_dense: true,
        };
        let encoded = point_cloud2(&cloud);

        let mut reader = CdrReader::new(&encoded.data);
        assert_eq!(reader.header(), cloud.header);
        assert_eq!(reader.u32(), 1);
        assert_eq!(reader.u32(), 4);
        assert_eq!(reader.u32(), 2);
        for expected in &cloud.fields {
            assert_eq!(&reader.string(), &expected.name);
            assert_eq!(reader.u32(), expected.offset);
            assert_eq!(reader.u8(), expected.datatype);
            assert_eq!(reader.u32(), expected.count);
        }
        assert!(!reader.boolean());
        assert_eq!(reader.u32(), 16);
        assert_eq!(reader.u32(), 64);
        assert_eq!(reader.bytes(), points);
        assert!(reader.boolean());
        assert!(reader.at_end());
    }

    #[test]
    fn imu_reads_back_all_six_arrays() {
        let message = Imu::unoriented(
            Header::new(7_000_000_123, "livox_imu"),
            [0.01, -0.02, 0.03],
            [0.1, 0.2, 9.81],
        );
        let encoded = imu(&message);

        let mut reader = CdrReader::new(&encoded.data);
        assert_eq!(reader.header(), message.header);
        assert_eq!(reader.f64_array::<4>(), [0.0, 0.0, 0.0, 1.0]);
        assert_eq!(reader.f64_array::<9>()[0], -1.0);
        assert_eq!(reader.f64_array::<3>(), [0.01, -0.02, 0.03]);
        assert_eq!(reader.f64_array::<9>(), [0.0; 9]);
        assert_eq!(reader.f64_array::<3>(), [0.1, 0.2, 9.81]);
        assert_eq!(reader.f64_array::<9>(), [0.0; 9]);
        assert!(reader.at_end());
    }

    #[test]
    fn a_tf_message_reads_back_every_transform() {
        let transforms = vec![
            TransformStamped {
                header: Header::new(1_000_000_000, "base_link"),
                child_frame_id: "camera_link".into(),
                translation: [0.1, 0.0, 0.2],
                rotation: [0.0, 0.0, 0.0, 1.0],
            },
            TransformStamped {
                header: Header::new(1_000_000_000, "base_link"),
                child_frame_id: "livox_frame".into(),
                translation: [0.0, 0.0, 0.3],
                rotation: [
                    0.0,
                    0.0,
                    std::f64::consts::FRAC_1_SQRT_2,
                    std::f64::consts::FRAC_1_SQRT_2,
                ],
            },
        ];
        let encoded = tf_message(&transforms);

        let mut reader = CdrReader::new(&encoded.data);
        assert_eq!(reader.u32(), 2);
        for expected in &transforms {
            assert_eq!(&reader.header(), &expected.header);
            assert_eq!(reader.string(), expected.child_frame_id);
            assert_eq!(reader.f64_array::<3>(), expected.translation);
            assert_eq!(reader.f64_array::<4>(), expected.rotation);
        }
        assert!(reader.at_end());
    }

    #[test]
    fn an_empty_tf_message_is_just_a_zero_count() {
        let encoded = tf_message(&[]);
        assert_eq!(encoded.data.len(), 8);
        let mut reader = CdrReader::new(&encoded.data);
        assert_eq!(reader.u32(), 0);
        assert!(reader.at_end());
    }

    #[test]
    fn a_compressed_image_hands_its_payload_through_untouched() {
        let jpeg = vec![0xFFu8, 0xD8, 0xFF, 0xE0, 0x00, 0x10];
        let image = CompressedImage {
            header: Header::new(3_000_000_000, "camera_color_optical_frame"),
            format: "jpeg".into(),
            data: jpeg.clone(),
        };
        let encoded = compressed_image(&image);

        let mut reader = CdrReader::new(&encoded.data);
        assert_eq!(reader.header(), image.header);
        assert_eq!(reader.string(), "jpeg");
        assert_eq!(reader.bytes(), jpeg);
        assert!(reader.at_end());
    }

    #[test]
    fn a_raw_image_keeps_its_stride_and_pixels() {
        let image = RawImage {
            header: Header::new(2_000_000_000, "camera_depth_optical_frame"),
            width: 4,
            height: 2,
            step: 8,
            is_bigendian: 0,
            encoding: "16UC1".into(),
            data: (0..16u8).collect(),
        };
        let encoded = raw_image(&image);
        assert_eq!(encoded.schema_name, "sensor_msgs/msg/Image");

        let mut reader = CdrReader::new(&encoded.data);
        assert_eq!(reader.header(), image.header);
        assert_eq!(reader.u32(), 2);
        assert_eq!(reader.u32(), 4);
        assert_eq!(reader.string(), "16UC1");
        assert_eq!(reader.u8(), 0);
        assert_eq!(reader.u32(), 8);
        assert_eq!(reader.bytes(), image.data);
        assert!(reader.at_end());
    }

    /// Every schema we emit has to name each of its dependencies, or a reader
    /// that resolves them (Foxglove does) drops the message.
    #[test]
    fn every_schema_defines_the_types_it_references() {
        let samples = [
            camera_info(&CameraInfo::pinhole(
                Header::new(0, "f"),
                1,
                1,
                1.0,
                1.0,
                0.0,
                0.0,
                crate::msgs::DistortionModel::PlumbBob,
                vec![0.0; 5],
                0.0,
            )),
            imu(&Imu::unoriented(Header::new(0, "f"), [0.0; 3], [0.0; 3])),
            tf_message(&[TransformStamped::identity("a", "b")]),
        ];
        for encoded in samples {
            assert!(
                encoded.schema_text.contains("MSG: builtin_interfaces/Time"),
                "{} is missing the Time definition",
                encoded.schema_name
            );
            assert!(
                encoded.schema_text.contains("MSG: std_msgs/Header"),
                "{} is missing the Header definition",
                encoded.schema_name
            );
        }
    }
}

// --- readers for the tools that consume a recording -------------------------
// `crate::heatmap` and `crate::video` read streams that other systems produce
// (Point-LIO odometry, RTAB-Map tf), so unlike the writers above these have
// to cope with bytes this program did not make.

/// The pose part of a `nav_msgs/Odometry` or `geometry_msgs/PoseStamped`; the
/// covariance and twist are skipped because nothing here draws them.
#[derive(Clone, Debug, PartialEq)]
pub struct Pose {
    pub header: Header,
    pub position: [f64; 3],
    pub orientation: [f64; 4],
}

/// A `sensor_msgs/PointCloud2` whose payload still borrows the message, so a
/// scan can be skimmed for xyz without copying its bytes.
#[derive(Clone, Debug, PartialEq)]
pub struct CloudView<'a> {
    pub header: Header,
    pub height: u32,
    pub width: u32,
    pub fields: Vec<crate::msgs::PointField>,
    pub is_bigendian: bool,
    pub point_step: u32,
    pub data: &'a [u8],
}

impl<'a> CdrReader<'a> {
    /// `new` asserts, which is right for bytes this program wrote and wrong for
    /// a stream off disk. Byte 1 of the encapsulation header is the endianness.
    pub fn little_endian(data: &'a [u8]) -> anyhow::Result<Self> {
        anyhow::ensure!(data.len() >= 4, "message is shorter than a CDR header");
        anyhow::ensure!(
            data[1] & 1 == 1,
            "message is big-endian CDR, which this recorder never writes and these tools do not read"
        );
        Ok(CdrReader { data, offset: 4 })
    }

    pub fn try_u8(&mut self) -> Option<u8> {
        (self.remaining() >= 1).then(|| self.u8())
    }

    pub fn try_u32(&mut self) -> Option<u32> {
        self.align(4);
        (self.remaining() >= 4).then(|| self.u32())
    }

    /// A `uint8[]` without the copy `bytes` makes.
    pub fn try_borrowed_bytes(&mut self) -> Option<&'a [u8]> {
        let length = self.try_u32()? as usize;
        let slice = self.data.get(self.offset..self.offset + length)?;
        self.offset += length;
        Some(slice)
    }
}

fn truncated(what: &str) -> anyhow::Error {
    anyhow::anyhow!("{what} is truncated")
}

pub fn decode_point_cloud2(data: &[u8]) -> anyhow::Result<CloudView<'_>> {
    let mut reader = CdrReader::little_endian(data)?;
    let short = || truncated("PointCloud2");
    let header = reader.try_header().ok_or_else(short)?;
    let height = reader.try_u32().ok_or_else(short)?;
    let width = reader.try_u32().ok_or_else(short)?;
    let field_count = reader.try_u32().ok_or_else(short)?;
    let mut fields = Vec::with_capacity(field_count.min(64) as usize);
    for _ in 0..field_count {
        fields.push(crate::msgs::PointField {
            name: reader.try_string().ok_or_else(short)?,
            offset: reader.try_u32().ok_or_else(short)?,
            datatype: reader.try_u8().ok_or_else(short)?,
            count: reader.try_u32().ok_or_else(short)?,
        });
    }
    let is_bigendian = reader.try_u8().ok_or_else(short)? != 0;
    let point_step = reader.try_u32().ok_or_else(short)?;
    reader.try_u32().ok_or_else(short)?; // row_step
    let data = reader.try_borrowed_bytes().ok_or_else(short)?;
    Ok(CloudView {
        header,
        height,
        width,
        fields,
        is_bigendian,
        point_step,
        data,
    })
}

pub fn decode_pose_stamped(data: &[u8]) -> anyhow::Result<Pose> {
    let mut reader = CdrReader::little_endian(data)?;
    let short = || truncated("PoseStamped");
    Ok(Pose {
        header: reader.try_header().ok_or_else(short)?,
        position: reader.try_f64_array().ok_or_else(short)?,
        orientation: reader.try_f64_array().ok_or_else(short)?,
    })
}

pub fn decode_odometry(data: &[u8]) -> anyhow::Result<Pose> {
    let mut reader = CdrReader::little_endian(data)?;
    let short = || truncated("Odometry");
    let header = reader.try_header().ok_or_else(short)?;
    reader.try_string().ok_or_else(short)?; // child_frame_id
    Ok(Pose {
        header,
        position: reader.try_f64_array().ok_or_else(short)?,
        orientation: reader.try_f64_array().ok_or_else(short)?,
    })
}

pub fn decode_tf_message(data: &[u8]) -> anyhow::Result<Vec<TransformStamped>> {
    let mut reader = CdrReader::little_endian(data)?;
    let short = || truncated("TFMessage");
    let count = reader.try_u32().ok_or_else(short)?;
    let mut transforms = Vec::with_capacity(count.min(256) as usize);
    for _ in 0..count {
        transforms.push(TransformStamped {
            header: reader.try_header().ok_or_else(short)?,
            child_frame_id: reader.try_string().ok_or_else(short)?,
            translation: reader.try_f64_array().ok_or_else(short)?,
            rotation: reader.try_f64_array().ok_or_else(short)?,
        });
    }
    Ok(transforms)
}

/// Writers for the two pose messages, so a test recording can carry the
/// streams the tools read. The covariances and twist go out as zeros.
pub fn odometry_pose(pose: &Pose, child_frame_id: &str) -> Encoded {
    let mut writer = CdrWriter::with_capacity(720);
    write_header(&mut writer, &pose.header);
    writer.string(child_frame_id);
    writer.f64_array(&pose.position);
    writer.f64_array(&pose.orientation);
    writer.f64_array(&[0.0; 36]);
    writer.f64_array(&[0.0; 6]);
    writer.f64_array(&[0.0; 36]);
    Encoded {
        schema_name: "nav_msgs/msg/Odometry",
        schema_text: format!(
            "std_msgs/Header header\n\
             string child_frame_id\n\
             geometry_msgs/PoseWithCovariance pose\n\
             geometry_msgs/TwistWithCovariance twist\n\n\
             ================================================================================\n\
             MSG: geometry_msgs/PoseWithCovariance\n\
             geometry_msgs/Pose pose\n\
             float64[36] covariance\n\n\
             ================================================================================\n\
             MSG: geometry_msgs/Pose\n\
             geometry_msgs/Point position\n\
             geometry_msgs/Quaternion orientation\n\n\
             ================================================================================\n\
             MSG: geometry_msgs/Point\n\
             float64 x\n\
             float64 y\n\
             float64 z\n\n\
             ================================================================================\n\
             MSG: geometry_msgs/TwistWithCovariance\n\
             geometry_msgs/Twist twist\n\
             float64[36] covariance\n\n\
             ================================================================================\n\
             MSG: geometry_msgs/Twist\n\
             geometry_msgs/Vector3 linear\n\
             geometry_msgs/Vector3 angular\n\
             {VECTOR3_MSG}{QUATERNION_MSG}{HEADER_MSG}\n{TIME_MSG}"
        ),
        data: writer.finish(),
    }
}

pub fn pose_stamped(pose: &Pose) -> Encoded {
    let mut writer = CdrWriter::with_capacity(96);
    write_header(&mut writer, &pose.header);
    writer.f64_array(&pose.position);
    writer.f64_array(&pose.orientation);
    Encoded {
        schema_name: "geometry_msgs/msg/PoseStamped",
        schema_text: format!(
            "std_msgs/Header header\n\
             geometry_msgs/Pose pose\n\n\
             ================================================================================\n\
             MSG: geometry_msgs/Pose\n\
             geometry_msgs/Point position\n\
             geometry_msgs/Quaternion orientation\n\n\
             ================================================================================\n\
             MSG: geometry_msgs/Point\n\
             float64 x\n\
             float64 y\n\
             float64 z\n\
             {QUATERNION_MSG}{HEADER_MSG}\n{TIME_MSG}"
        ),
        data: writer.finish(),
    }
}

#[cfg(test)]
mod decode_tests {
    use super::*;
    use crate::msgs::{PointField, POINT_FIELD_FLOAT32};

    fn a_pose() -> Pose {
        Pose {
            header: Header::new(9_000_000_500, "odom"),
            position: [1.5, -2.0, 0.25],
            orientation: [0.0, 0.0, std::f64::consts::FRAC_1_SQRT_2, std::f64::consts::FRAC_1_SQRT_2],
        }
    }

    #[test]
    fn odometry_decodes_the_pose_and_ignores_the_covariance_and_twist() {
        let encoded = odometry_pose(&a_pose(), "base_link");
        // Body: header 12 + "odom\0" 5 = 17, padded to 20; "base_link\0" 4 + 10 = 34,
        // padded to 40; then 7 + 36 + 6 + 36 doubles.
        assert_eq!(encoded.data.len(), 4 + 40 + 85 * 8);
        assert_eq!(decode_odometry(&encoded.data).unwrap(), a_pose());
    }

    #[test]
    fn pose_stamped_decodes_the_pose() {
        let encoded = pose_stamped(&a_pose());
        assert_eq!(decode_pose_stamped(&encoded.data).unwrap(), a_pose());
    }

    #[test]
    fn a_tf_message_decodes_every_transform_it_was_written_with() {
        let transforms = vec![
            TransformStamped::identity("odom", "base_link"),
            TransformStamped {
                header: Header::new(4_000_000_000, "base_link"),
                child_frame_id: "livox_frame".into(),
                translation: [0.1, 0.2, 0.3],
                rotation: [0.0, 1.0, 0.0, 0.0],
            },
        ];
        let encoded = tf_message(&transforms);
        assert_eq!(decode_tf_message(&encoded.data).unwrap(), transforms);
        assert!(decode_tf_message(&encoded.data[..encoded.data.len() - 3]).is_err());
    }

    #[test]
    fn a_point_cloud_view_borrows_its_payload_and_keeps_the_field_offsets() {
        let cloud = crate::msgs::PointCloud2 {
            header: Header::new(5_000_000_000, "livox_frame"),
            height: 1,
            width: 2,
            fields: ["x", "y", "z"]
                .iter()
                .enumerate()
                .map(|(index, name)| PointField {
                    name: name.to_string(),
                    offset: index as u32 * 4,
                    datatype: POINT_FIELD_FLOAT32,
                    count: 1,
                })
                .collect(),
            is_bigendian: false,
            point_step: 16,
            row_step: 32,
            data: (0..32u8).collect(),
            is_dense: true,
        };
        let encoded = point_cloud2(&cloud);
        let view = decode_point_cloud2(&encoded.data).unwrap();
        assert_eq!(view.header, cloud.header);
        assert_eq!(view.width, 2);
        assert_eq!(view.fields, cloud.fields);
        assert_eq!(view.point_step, 16);
        assert_eq!(view.data, &cloud.data[..]);
        assert!(!view.is_bigendian);
        assert!(decode_point_cloud2(&encoded.data[..40]).is_err());
    }

    #[test]
    fn big_endian_cdr_is_refused_with_a_reason() {
        let mut encoded = pose_stamped(&a_pose()).data;
        encoded[1] = 0x00;
        let error = decode_pose_stamped(&encoded).unwrap_err().to_string();
        assert!(error.contains("big-endian"), "{error}");
    }
}

/// The full `nav_msgs/Odometry`, as the post-processor writes it. Covariances
/// go out as zeros.
pub fn odometry(odometry: &Odometry) -> Encoded {
    let mut writer = CdrWriter::with_capacity(800);
    write_header(&mut writer, &odometry.header);
    writer.string(&odometry.child_frame_id);
    writer.f64_array(&odometry.position);
    writer.f64_array(&odometry.orientation);
    writer.f64_array(&[0.0; 36]);
    writer.f64_array(&odometry.linear_velocity);
    writer.f64_array(&odometry.angular_velocity);
    writer.f64_array(&[0.0; 36]);
    Encoded {
        schema_name: crate::msgs::ODOMETRY_TYPE,
        schema_text: odometry_pose(
            &Pose {
                header: Header::default(),
                position: [0.0; 3],
                orientation: [0.0, 0.0, 0.0, 1.0],
            },
            "",
        )
        .schema_text,
        data: writer.finish(),
    }
}

/// Only the header, which is all that is needed to learn a stream's frame and
/// stamp. `None` for a payload that is not little-endian CDR or is too short.
pub fn decode_header(payload: &[u8]) -> Option<Header> {
    if payload.len() < 4 || payload[..4] != [0x00, 0x01, 0x00, 0x00] {
        return None;
    }
    CdrReader::new(payload).try_header()
}

/// The whole `nav_msgs/Odometry`, twist included; `decode_odometry` above
/// stops at the pose because the drawing tools need no more.
pub fn decode_odometry_message(payload: &[u8]) -> Option<Odometry> {
    if payload.len() < 8 || payload[..4] != [0x00, 0x01, 0x00, 0x00] {
        return None;
    }
    let mut reader = CdrReader::new(payload);
    let header = reader.try_header()?;
    let child_frame_id = reader.try_string()?;
    let position = reader.try_f64_array()?;
    let orientation = reader.try_f64_array()?;
    let _pose_covariance: [f64; 36] = reader.try_f64_array()?;
    let linear_velocity = reader.try_f64_array()?;
    let angular_velocity = reader.try_f64_array()?;
    Some(Odometry {
        header,
        child_frame_id,
        position,
        orientation,
        linear_velocity,
        angular_velocity,
    })
}

#[cfg(test)]
mod odometry_message_tests {
    use super::*;

    #[test]
    fn an_odometry_message_round_trips_and_carries_its_schema() {
        let original = Odometry {
            header: Header::new(1_700_000_000_123_456_789, "odom"),
            child_frame_id: "base_link".into(),
            position: [1.0, -2.0, 0.5],
            orientation: [0.0, 0.0, std::f64::consts::FRAC_1_SQRT_2, std::f64::consts::FRAC_1_SQRT_2],
            linear_velocity: [0.3, 0.0, 0.0],
            angular_velocity: [0.0, 0.0, 0.1],
        };
        let encoded = odometry(&original);
        assert_eq!(encoded.schema_name, "nav_msgs/msg/Odometry");
        assert!(encoded.schema_text.contains("MSG: geometry_msgs/TwistWithCovariance"));
        assert_eq!(decode_odometry_message(&encoded.data).unwrap(), original);
        assert_eq!(decode_header(&encoded.data).unwrap(), original.header);
        // The pose-only reader agrees with the full one.
        let pose = decode_odometry(&encoded.data).unwrap();
        assert_eq!(pose.position, original.position);
        assert_eq!(pose.header, original.header);
    }
}
