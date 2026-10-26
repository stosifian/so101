import pytest

from so101.hw.registers import REGISTERS, decode, encode


def test_no_overlapping_addresses():
    spans = sorted((r.addr, r.addr + r.size, r.name) for r in REGISTERS.values())
    for (a0, a1, n0), (b0, _, n1) in zip(spans, spans[1:]):
        assert a1 <= b0, f"{n0} overlaps {n1}"


@pytest.mark.parametrize(
    "name, raw, value",
    [
        ("Present_Load", 0x0064, 100),  # +10.0 %
        ("Present_Load", 0x0464, -100),  # bit 10 = sign
        ("Offset", 0x0800 | 37, -37),  # bit 11 = sign
        ("Present_Speed", 0x8000 | 250, -250),
        ("Present_Voltage", 119, 119),  # unsigned
    ],
)
def test_decode_sign_magnitude(name, raw, value):
    assert decode(REGISTERS[name], raw) == value


@pytest.mark.parametrize("name", ["Present_Load", "Offset", "Goal_Position", "Goal_Speed"])
@pytest.mark.parametrize("value", [-500, -1, 0, 1, 500])
def test_encode_decode_roundtrip(name, value):
    reg = REGISTERS[name]
    assert decode(reg, encode(reg, value)) == value


def test_encode_rejects_out_of_range():
    with pytest.raises(ValueError):
        encode(REGISTERS["Offset"], 2048)  # magnitude must fit in 11 bits
