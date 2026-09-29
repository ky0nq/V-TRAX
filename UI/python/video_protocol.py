"""HUDV v1: bounded, out-of-order UDP RGB888/RGB565 reassembly (no Qt dependency)."""
import struct
import time
from dataclasses import dataclass

HEADER = struct.Struct('!4sBBHHHHHIII')
CHUNK_BYTES = 1200
MAX_WIDTH, MAX_HEIGHT = 1280, 720

@dataclass(frozen=True)
class Frame:
    width: int
    height: int
    pixels: bytes
    sequence: int
    format: int = 1

def packetize(pixels, width, height, sequence, session=1, fmt=3):
    if fmt not in (1, 3):
        raise ValueError("Unsupported format")
    total = width * height * (3 if fmt == 1 else 2)
    if len(pixels) != total or not (0 < width <= MAX_WIDTH and 0 < height <= MAX_HEIGHT):
        raise ValueError('Invalid RGB888 frame')
    count = (total + CHUNK_BYTES - 1) // CHUNK_BYTES
    for index in range(count):
        part = pixels[index*CHUNK_BYTES:(index+1)*CHUNK_BYTES]
        yield HEADER.pack(b'HUDV', 1, fmt, width, height, index, count, len(part),
                          sequence & 0xffffffff, total, session & 0xffffffff) + part

def newer(a, b):
    return 0 < ((a-b) & 0xffffffff) < 0x80000000

class Reassembler:
    def __init__(self, timeout=1.5):
        self.timeout = timeout
        self.pending = {}
        self.session = None
        self.last_sequence = None
        self.last_input = 0.0

    def expire(self, now=None):
        now = time.monotonic() if now is None else now
        for key in list(self.pending):
            if now - self.pending[key][0] > self.timeout:
                del self.pending[key]

    def feed(self, packet, now=None):
        now = time.monotonic() if now is None else now
        self.expire(now)
        if len(packet) < HEADER.size:
            return None
        magic, version, fmt, w, h, index, count, size, seq, total, session = HEADER.unpack_from(packet)
        if (magic != b'HUDV' or version != 1 or fmt not in (1, 3) or
            not 0 < w <= MAX_WIDTH or not 0 < h <= MAX_HEIGHT or
            total != w*h*(3 if fmt == 1 else 2) or count != (total+CHUNK_BYTES-1)//CHUNK_BYTES or
            index >= count or size != min(CHUNK_BYTES, total-index*CHUNK_BYTES) or
            len(packet) != HEADER.size+size):
            return None
        # Session changes only after the previous sender has gone quiet.
        # This avoids late packets from an old boot resetting the new stream.
        if self.session != session:
            if self.session is not None and now-self.last_input < 1.2:
                return None
            self.pending.clear()
            self.last_sequence = None
            self.session = session
        self.last_input = now
        if self.last_sequence is not None and not newer(seq, self.last_sequence):
            return None
        if seq not in self.pending:
            if len(self.pending) >= 3:
                del self.pending[min(self.pending, key=lambda k: self.pending[k][0])]
            self.pending[seq] = [now, w, h, {}, total, fmt]
        record = self.pending[seq]
        if record[1:3] != [w, h] or record[4] != total or record[5] != fmt:
            return None
        record[3].setdefault(index, packet[HEADER.size:])
        if len(record[3]) != count:
            return None
        frame = Frame(w, h, b''.join(record[3][i] for i in range(count)), seq, fmt)
        self.last_sequence = seq
        self.pending = {k:v for k,v in self.pending.items() if newer(k, seq)}
        return frame
