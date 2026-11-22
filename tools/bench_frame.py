"""SatUVTRX A bench v1 ground-side frame and IQ generator (Python stdlib)."""
import argparse
import binascii
import math
from pathlib import Path
import struct

PREFIX = bytes.fromhex("555555551acffc1d")


def encode(kind, sequence, data):
    if not 1 <= kind <= 7 or not 0 <= sequence <= 65535 or len(data) > 8:
        raise ValueError("kind 1..7, sequence 0..65535, payload 0..8 bytes required")
    body = struct.pack(">BBHB8s", 1, kind, sequence, len(data), data)
    return PREFIX + body + struct.pack(">H", binascii.crc_hqx(body, 0xffff))


def decode(wire):
    if len(wire) != 23 or not wire.startswith(PREFIX):
        raise ValueError("invalid frame length or prefix")
    body = wire[8:-2]
    version, kind, seq, size, payload = struct.unpack(">BBHB8s", body)
    if version != 1 or size > 8 or encode(kind, seq, payload[:size]) != wire:
        raise ValueError("invalid format, padding or CRC")
    return kind, seq, payload[:size]


def samples(wire, rate, intermediate_hz):
    sps = 48000 // rate
    signs = []
    sign = 1
    for byte in wire:
        for bit in range(7, -1, -1):
            if byte & (1 << bit):
                sign = -sign
            signs.append(sign)

    def pulse(t):
        if t == 0:
            return 1.0
        if abs(t) >= 4 or abs(t) == 2:
            return 0.0
        return math.sin(math.pi*t)/(math.pi*t) * math.cos(math.pi*t/4)/(1-(t/2)**2)

    for n in range((len(signs)+8)*sps):
        symbol, sub = divmod(n, sps)
        amplitude = sum(signs[b]*pulse(symbol-4+sub/sps-b)
                        for b in range(max(0,symbol-8), min(symbol+1,len(signs))))
        angle = 2*math.pi*intermediate_hz*n/48000
        yield struct.pack("<hh", round(8192*amplitude*math.cos(angle)),
                          round(8192*amplitude*math.sin(angle)))


def demodulate(stream, rate, intermediate_hz):
    """Find CRC-valid packets at any integer symbol phase in signed 16-bit IQ."""
    sps = 48000 // rate
    window = [0j] * sps
    previous = [0j] * sps
    shift = [0] * sps
    remaining = [0] * sps
    bodies = [0] * sps
    total = 0j
    n = 0
    last_match = -sps
    while raw := stream.read(65536):
        if len(raw) % 4:
            raise ValueError("truncated IQ sample")
        for i, q in struct.iter_unpack("<hh", raw):
            phase = n % sps
            angle = -2*math.pi*intermediate_hz*n/48000
            sample = complex(i, q)*complex(math.cos(angle), math.sin(angle))
            total += sample-window[phase]
            window[phase] = sample
            bit = (total*previous[phase].conjugate()).real < 0
            previous[phase] = total
            shift[phase] = ((shift[phase]<<1) | bit) & 0xffffffff
            if remaining[phase]:
                bodies[phase] = (bodies[phase]<<1) | bit
                remaining[phase] -= 1
                if not remaining[phase] and n-last_match >= sps:
                    wire = PREFIX + bodies[phase].to_bytes(15, "big")
                    try:
                        packet = decode(wire)
                    except ValueError:
                        pass
                    else:
                        last_match = n
                        yield packet
            elif shift[phase] == 0x1acffc1d:
                remaining[phase] = 120
                bodies[phase] = 0
            n += 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="action", required=True)
    tx = commands.add_parser("encode")
    tx.add_argument("kind", type=int)
    tx.add_argument("sequence", type=int)
    tx.add_argument("payload", help="hexadecimal, up to 8 bytes")
    tx.add_argument("--iq", type=Path)
    tx.add_argument("--rate", type=int, choices=[1200, 9600], default=1200)
    tx.add_argument("--if-hz", type=int, default=0, help="0 for an SDR, 12000 for codec-input simulation")
    rx = commands.add_parser("decode")
    rx.add_argument("frame")
    iq = commands.add_parser("demod")
    iq.add_argument("input", type=Path)
    iq.add_argument("--rate", type=int, choices=[1200,9600], default=9600)
    iq.add_argument("--if-hz", type=int, default=0)
    args = parser.parse_args()
    if args.action == "demod":
        with args.input.open("rb") as stream:
            for kind, seq, payload in demodulate(stream,args.rate,args.if_hz):
                print(f"kind={kind} sequence={seq} payload={payload.hex()}")
        return
    if args.action == "decode":
        kind, seq, payload = decode(bytes.fromhex(args.frame))
        print(f"kind={kind} sequence={seq} payload={payload.hex()}")
    else:
        wire = encode(args.kind, args.sequence, bytes.fromhex(args.payload))
        print(wire.hex())
        if args.iq:
            args.iq.write_bytes(b"".join(samples(wire, args.rate, args.if_hz)))


if __name__ == "__main__":
    main()
