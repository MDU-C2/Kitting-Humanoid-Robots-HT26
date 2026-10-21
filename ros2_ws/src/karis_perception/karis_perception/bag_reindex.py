"""Rebuild the index of an unindexed ROS 1 / RealSense .bag (like `rosbag reindex`, without ROS 1).

    ros2 run karis_perception bag_reindex in.bag [-o out.bag]    (default: in.reindexed.bag)

Format: ROS bag 2.0. lz4 chunks need the lz4 package. A truncated last chunk is dropped.
"""
import argparse
import bz2
import struct
import sys
from pathlib import Path

MAGIC = b"#ROSBAG V2.0\n"
OP_MSG, OP_BAG_HEADER, OP_INDEX, OP_CHUNK, OP_CHUNK_INFO, OP_CONN = 0x02, 0x03, 0x04, 0x05, 0x06, 0x07
BAG_HEADER_LEN = 4096


class BagError(RuntimeError):
    pass


def parse_header(buf):
    """Record header bytes -> {name: raw value bytes}."""
    fields, i = {}, 0
    while i + 4 <= len(buf):
        n = struct.unpack_from("<I", buf, i)[0]
        i += 4
        f = buf[i:i + n]
        i += n
        k, _, v = f.partition(b"=")
        fields[k.decode()] = v
    return fields


def make_header(fields):
    out = b""
    for k, v in fields.items():
        f = k.encode() + b"=" + v
        out += struct.pack("<I", len(f)) + f
    return out


def record(fields, data=b""):
    h = make_header(fields)
    return struct.pack("<I", len(h)) + h + struct.pack("<I", len(data)) + data


def read_record(f):
    """(header dict, data bytes, start offset) or None at a clean or cut-off end."""
    pos = f.tell()
    b = f.read(4)
    if len(b) < 4:
        return None
    hlen = struct.unpack("<I", b)[0]
    h = f.read(hlen)
    b = f.read(4)
    if len(h) < hlen or len(b) < 4:
        return None
    dlen = struct.unpack("<I", b)[0]
    d = f.read(dlen)
    if len(d) < dlen:
        return None
    return parse_header(h), d, pos


def iter_records(buf):
    i = 0
    while i + 4 <= len(buf):
        start = i
        hlen = struct.unpack_from("<I", buf, i)[0]
        i += 4
        if i + hlen + 4 > len(buf):
            return
        h = parse_header(buf[i:i + hlen])
        i += hlen
        dlen = struct.unpack_from("<I", buf, i)[0]
        i += 4
        if i + dlen > len(buf):
            return
        yield h, buf[i:i + dlen], start
        i += dlen


def decompress(kind, data, size):
    if kind == b"none":
        return data
    if kind == b"bz2":
        return bz2.decompress(data)
    if kind == b"lz4":
        try:
            import lz4.frame
        except ImportError:
            raise BagError("this bag uses lz4 compression: pip install lz4") from None
        return lz4.frame.decompress(data)
    raise BagError(f"unknown chunk compression {kind!r}")


def op(h):
    return h["op"][0]


def u32(v):
    return struct.unpack("<I", v)[0]


def time_key(v):
    sec, nsec = struct.unpack("<II", v)
    return sec * 1_000_000_000 + nsec


def reindex(src, dst, log=print):
    src, dst = Path(src), Path(dst)
    with open(src, "rb") as f:
        if f.read(len(MAGIC)) != MAGIC:
            raise BagError(f"{src} is not a ROS bag 2.0 file (RealSense .bag)")
        first = read_record(f)
        if first is None or op(first[0]) != OP_BAG_HEADER:
            raise BagError(f"{src}: no bag header record")
        conns = {}          # conn id -> (header fields, data) of its connection record
        chunks = []         # (raw chunk record bytes, {conn: [(time, offset)]}, start, end)
        n_msgs, cut = 0, False
        while True:
            r = read_record(f)
            if r is None:
                cut = f.tell() < src.stat().st_size
                break
            h, d, pos = r
            o = op(h)
            if o == OP_CHUNK:
                try:
                    raw = decompress(h["compression"], d, u32(h["size"]))
                except Exception as e:      # truncated or corrupt last chunk
                    log(f"  chunk at byte {pos}: {e}; dropped")
                    cut = True
                    break
                index, t0, t1 = {}, None, None
                for ih, idata, ipos in iter_records(raw):
                    io = op(ih)
                    if io == OP_CONN:
                        conns.setdefault(u32(ih["conn"]), (ih, idata))
                    elif io == OP_MSG:
                        t = ih["time"]
                        index.setdefault(u32(ih["conn"]), []).append((t, ipos))
                        tk = time_key(t)
                        t0 = t if t0 is None or tk < time_key(t0) else t0
                        t1 = t if t1 is None or tk > time_key(t1) else t1
                        n_msgs += 1
                if not index:
                    continue
                end = f.tell()
                f.seek(pos)
                chunks.append((f.read(end - pos), index, t0, t1))
                f.seek(end)
            elif o == OP_CONN:
                conns.setdefault(u32(h["conn"]), (h, d))
            # index and chunk info records are rebuilt
    if not chunks:
        raise BagError(f"{src}: no complete chunk with messages found")

    tmp = dst.with_suffix(dst.suffix + ".part")
    with open(tmp, "wb") as out:
        out.write(MAGIC)
        out.write(b"\0" * BAG_HEADER_LEN)                        # bag header, written at the end
        infos = []
        for raw, index, t0, t1 in chunks:
            chunk_pos = out.tell()
            out.write(raw)
            for c, entries in sorted(index.items()):
                data = b"".join(t + struct.pack("<I", off) for t, off in entries)
                out.write(record({"op": bytes([OP_INDEX]), "ver": struct.pack("<I", 1),
                                  "conn": struct.pack("<I", c),
                                  "count": struct.pack("<I", len(entries))}, data))
            infos.append((chunk_pos, t0, t1, {c: len(e) for c, e in index.items()}))
        index_pos = out.tell()
        for c in sorted(conns):
            h, d = conns[c]
            h = dict(h)
            h["op"] = bytes([OP_CONN])
            out.write(record(h, d))
        for chunk_pos, t0, t1, counts in infos:
            data = b"".join(struct.pack("<II", c, n) for c, n in sorted(counts.items()))
            out.write(record({"op": bytes([OP_CHUNK_INFO]), "ver": struct.pack("<I", 1),
                              "chunk_pos": struct.pack("<Q", chunk_pos),
                              "start_time": t0, "end_time": t1,
                              "count": struct.pack("<I", len(counts))}, data))
        hdr = make_header({"op": bytes([OP_BAG_HEADER]), "index_pos": struct.pack("<Q", index_pos),
                           "conn_count": struct.pack("<I", len(conns)),
                           "chunk_count": struct.pack("<I", len(chunks))})
        pad = BAG_HEADER_LEN - 8 - len(hdr)
        out.seek(len(MAGIC))
        out.write(struct.pack("<I", len(hdr)) + hdr + struct.pack("<I", pad) + b" " * pad)
    tmp.replace(dst)
    log(f"Reindexed {src.name}: {len(chunks)} chunks, {n_msgs} messages, {len(conns)} topics"
        + (" (the cut-off end was dropped)" if cut else "") + f" -> {dst}")
    return dst


def is_indexed(path):
    with open(path, "rb") as f:
        if f.read(len(MAGIC)) != MAGIC:
            return True                       # not a bag 2.0 file
        r = read_record(f)
    return r is not None and struct.unpack("<Q", r[0].get("index_pos", b"\0" * 8))[0] != 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("-o", "--output", help="default: <name>.reindexed.bag next to the input")
    a = ap.parse_args()
    src = Path(a.bag)
    dst = Path(a.output) if a.output else src.with_name(src.stem + ".reindexed.bag")
    try:
        reindex(src, dst)
    except BagError as e:
        sys.exit(f"bag_reindex: {e}")


if __name__ == "__main__":
    main()
