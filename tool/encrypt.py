import sys

def encrypt_block(dec: bytes) -> bytes:
    if len(dec) != 0x100:
        raise ValueError("Input block must be exactly 256 bytes")

    enc = bytearray(0x100)
    for i in range(0x100):
        t = (i * 0x11) & 0xFF
        enc[i] = dec[t]

    return bytes(enc)


def encrypt_file(
    input_path: str,
    output_path: str,
    offset: int
):
    with open(input_path, "rb") as f:
        data = bytearray(f.read())

    if offset + 0x100 > len(data):
        raise ValueError("File is too small for the given offset")

    dec = bytes(data[offset:offset + 0x100])
    enc = encrypt_block(dec)

    data[offset:offset + 0x100] = enc

    with open(output_path, "wb") as f:
        f.write(data)


if __name__ == "__main__":
    if len(sys.argv) != 4:
        print(f"Usage: {sys.argv[0]} <input_file> <output_file> <offset>")
        print("Offset can be decimal or hex (e.g. 0x200)")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2]
    offset = int(sys.argv[3], 0)  # auto-detect hex or decimal

    encrypt_file(input_file, output_file, offset)
    print("Encryption complete.")
