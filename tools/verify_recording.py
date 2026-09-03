"""Checks an mcap recording against what the sensors were supposed to deliver.

Reads the CDR payloads directly rather than through a ROS message library, so it
runs anywhere the `mcap` package is installed and needs no message definitions.
Every ROS message this recorder writes begins with a std_msgs/Header, which is
all the timestamp checks need.

The distinction that matters here: `log_time` is stamped by the recorder when a
message arrives, while the header stamp comes from the sensor. Comparing rates
derived from each is the only way to tell a real sensor clock from a host clock
copied into the header, because the two agree on average but not in their drift.
"""

import argparse
import struct
import sys
from dataclasses import dataclass, field

from mcap.reader import make_reader

NANOSECONDS_PER_SECOND = 1_000_000_000

# CDR encapsulation header, then the Header's sec/nanosec.
STAMP_OFFSET = 4

# A rate measured from timestamps lands a hair either side of the target, so an
# exact comparison fails a sensor that hit its rate. Small enough that a genuine
# shortfall, like an IMU running 1.3% under its requested rate, still reports.
RATE_SHORTFALL_TOLERANCE = 0.001


def header_stamp_nanos(payload):
    if len(payload) < STAMP_OFFSET + 8:
        return None
    seconds, nanoseconds = struct.unpack_from("<iI", payload, STAMP_OFFSET)
    return seconds * NANOSECONDS_PER_SECOND + nanoseconds


def point_count(payload):
    """width * height for a PointCloud2, or None if it does not parse as one."""
    frame_id_length_at = STAMP_OFFSET + 8
    if len(payload) < frame_id_length_at + 4:
        return None
    (frame_id_length,) = struct.unpack_from("<I", payload, frame_id_length_at)
    after_frame_id = frame_id_length_at + 4 + frame_id_length
    # CDR aligns every primitive to its own size.
    aligned = after_frame_id + (-after_frame_id % 4)
    if len(payload) < aligned + 8:
        return None
    height, width = struct.unpack_from("<II", payload, aligned)
    return height * width


def rate(count, span_nanos):
    if count < 2 or span_nanos <= 0:
        return 0.0
    return (count - 1) * NANOSECONDS_PER_SECOND / span_nanos


@dataclass
class Topic:
    schema: str = ""
    count: int = 0
    log_first: int = 0
    log_last: int = 0
    log_times: list = field(default_factory=list)
    stamps: list = field(default_factory=list)
    points: list = field(default_factory=list)


def collect(path):
    topics = {}
    with open(path, "rb") as recording:
        reader = make_reader(recording)
        for schema, channel, message in reader.iter_messages():
            topic = topics.setdefault(channel.topic, Topic())
            topic.schema = schema.name if schema else ""
            if topic.count == 0:
                topic.log_first = message.log_time
            topic.count += 1
            topic.log_last = message.log_time
            topic.log_times.append(message.log_time)
            stamp = header_stamp_nanos(message.data)
            if stamp is not None:
                topic.stamps.append(stamp)
            if topic.schema.endswith("PointCloud2"):
                points = point_count(message.data)
                if points is not None:
                    topic.points.append(points)
    return topics


def largest_gap_nanos(log_times):
    """The worst inter-arrival delay, and the typical one to judge it against."""
    if len(log_times) < 3:
        return 0, 0
    deltas = sorted(b - a for a, b in zip(log_times, log_times[1:]))
    return deltas[-1], deltas[len(deltas) // 2]


def check(path, expectations, max_rate_error_percent):
    topics = collect(path)
    failures = []
    # A stream that dies halfway through still has a perfect rate over the
    # messages it did produce, so every topic is judged against the span of the
    # whole recording rather than against its own.
    recording_start = min(topic.log_first for topic in topics.values())
    recording_end = max(topic.log_last for topic in topics.values())
    print(f"{path}\n")
    header = f"{'topic':44s} {'count':>7s} {'hz':>8s} {'hdr hz':>8s} {'err %':>7s}"
    print(header)
    print("-" * len(header))

    for name in sorted(topics):
        topic = topics[name]
        log_rate = rate(topic.count, topic.log_last - topic.log_first)
        stamps = topic.stamps
        stamp_rate = rate(len(stamps), stamps[-1] - stamps[0]) if len(stamps) >= 2 else 0.0
        error_percent = abs(stamp_rate - log_rate) / log_rate * 100 if log_rate else 0.0
        print(
            f"{name:44s} {topic.count:7d} {log_rate:8.2f} {stamp_rate:8.2f} {error_percent:7.3f}"
        )

        # A camera_info published once has no rate to check and no drift to show.
        if topic.count < 2:
            continue

        widest_gap, typical_gap = largest_gap_nanos(topic.log_times)
        stopped_early = recording_end - topic.log_last
        started_late = topic.log_first - recording_start
        for when, delay in (("stopped", stopped_early), ("started", started_late)):
            # Ten periods of silence at the edge is a dead stream, not jitter.
            if typical_gap and delay > max(10 * typical_gap, NANOSECONDS_PER_SECOND):
                failures.append(
                    f"{name}: {when} {delay / NANOSECONDS_PER_SECOND:.1f}s away from the"
                    f" edge of the recording, so it was not running the whole time"
                )
        if typical_gap and widest_gap > max(10 * typical_gap, NANOSECONDS_PER_SECOND):
            failures.append(
                f"{name}: went quiet for {widest_gap / NANOSECONDS_PER_SECOND:.1f}s"
                f" mid-recording, against a typical"
                f" {typical_gap / NANOSECONDS_PER_SECOND * 1000:.1f}ms between messages"
            )

        # An arrival time copied into the header would satisfy every check above
        # while carrying none of the sensor's own timing.
        if stamps and all(stamp == log for stamp, log in zip(stamps, topic.log_times)):
            failures.append(
                f"{name}: every header stamp equals its arrival time, so the header"
                f" carries the host clock rather than the sensor's"
            )

        out_of_order = sum(1 for a, b in zip(stamps, stamps[1:]) if b <= a)
        if out_of_order:
            duplicates = sum(1 for a, b in zip(stamps, stamps[1:]) if b == a)
            failures.append(
                f"{name}: {out_of_order} header stamps are not increasing"
                f" ({duplicates} of them exact duplicates)"
            )
        if stamp_rate and error_percent > max_rate_error_percent:
            failures.append(
                f"{name}: header rate {stamp_rate:.3f} Hz differs from arrival rate"
                f" {log_rate:.3f} Hz by {error_percent:.3f}%, over the"
                f" {max_rate_error_percent}% allowed"
            )
        if topic.points:
            empty = sum(1 for count in topic.points if count == 0)
            mean = sum(topic.points) / len(topic.points)
            smallest = min(topic.points)
            print(f"{'':44s} points: mean {mean:.0f}, min {smallest}, empty {empty}")
            if empty:
                failures.append(f"{name}: {empty} clouds have no points in them")

        wanted = expectations.get(name)
        if wanted is not None and log_rate < wanted * (1 - RATE_SHORTFALL_TOLERANCE):
            failures.append(f"{name}: {log_rate:.2f} Hz is below the {wanted} Hz expected")

    for name in expectations:
        if name not in topics:
            failures.append(f"{name}: expected in the recording, but not present at all")

    print()
    if failures:
        for failure in failures:
            print(f"FAIL  {failure}")
        return 1
    print("all checks passed")
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording")
    parser.add_argument(
        "--expect",
        action="append",
        default=[],
        metavar="TOPIC=HZ",
        help="minimum acceptable rate for a topic; repeatable",
    )
    parser.add_argument(
        "--max-rate-error-percent",
        type=float,
        default=0.5,
        help="how far a header-derived rate may differ from the arrival rate",
    )
    arguments = parser.parse_args()

    expectations = {}
    for pair in arguments.expect:
        topic, _, hz = pair.partition("=")
        expectations[topic] = float(hz)

    return check(arguments.recording, expectations, arguments.max_rate_error_percent)


if __name__ == "__main__":
    sys.exit(main())
