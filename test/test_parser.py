"""Tests for NMEA sentence parsing, in particular talker ID handling."""

# parser.py does `import rclpy` but uses `rclpy.logging` at module scope, which
# is only populated once something else imports that submodule. Import it here
# so the parser module can be imported on its own.
import rclpy.logging  # noqa: F401

import pytest

from libnmea_navsat_driver.parser import parse_nmea_sentence

GGA = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"
RMC = "$GNRMC,123519.00,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*5A"


@pytest.mark.parametrize("sentence,expected_type", [
    (GGA, "GGA"),
    (RMC, "RMC"),
    ("$IIHDT,11.4,T*16", "HDT"),   # integrated instrumentation talker
    ("$HEHDT,11.4,T*1B", "HDT"),   # gyro compass talker
    ("$HDT,11.4,T*16", "HDT"),     # no talker ID at all
])
def test_supported_sentence_parsed_for_any_talker(sentence, expected_type):
    parsed = parse_nmea_sentence(sentence)

    assert parsed is not False, "sentence was rejected before reaching the parse map"
    assert expected_type in parsed


def test_heading_extracted_from_non_gnss_talker():
    parsed = parse_nmea_sentence("$IIHDT,11.4,T*16")

    assert parsed["HDT"]["heading"] == pytest.approx(11.4)


def test_bare_hdt_resolves_to_hdt_and_not_a_truncated_type():
    """A sentence with no talker ID must not be mis-sliced into a bogus type."""
    parsed = parse_nmea_sentence("$HDT,11.4,T*16")

    assert list(parsed) == ["HDT"]


@pytest.mark.parametrize("sentence", [
    "$IIVWR,55.93,R,1.56,N,0.80,M,2.88,K*7B",     # well-formed, type not supported
    "$GPGGA,123519,4807.038,N,01131.000,E,1,08",  # no checksum
    "GPGGA,123519*47",                            # no leading $
    "$GP,123519*47",                              # no sentence type
    "not an nmea sentence",
    "",
])
def test_unparseable_sentences_are_rejected(sentence):
    assert parse_nmea_sentence(sentence) is False
