import struct


class WebSocketProtocol:
    def parse_headers(self, data):
        headers = {}
        try:
            lines = data.decode('utf-8').split('\r\n')
        except Exception:
            return headers
        for line in lines[1:]:
            if ': ' in line:
                key, _, val = line.partition(': ')
                headers[key] = val
        return headers

    def decode_frame(self, data):
        """Return (payload_str, remaining_bytes) or (None, data) if frame incomplete."""
        try:
            if len(data) < 2:
                return None, data

            second_byte = data[1]
            masked = bool(second_byte & 0x80)
            length = second_byte & 0x7F
            idx = 2

            if length == 126:
                if len(data) < 4:
                    return None, data
                length = struct.unpack("!H", data[2:4])[0]
                idx = 4
            elif length == 127:
                if len(data) < 10:
                    return None, data
                length = struct.unpack("!Q", data[2:10])[0]
                idx = 10

            if masked:
                if len(data) < idx + 4 + length:
                    return None, data
                masks = data[idx: idx + 4]
                idx += 4
                decoded = bytearray(
                    data[idx + i] ^ masks[i % 4] for i in range(length)
                )
            else:
                if len(data) < idx + length:
                    return None, data
                decoded = data[idx: idx + length]

            remaining = data[idx + length:]
            return decoded.decode('utf-8'), remaining
        except Exception:
            return None, b""

    def encode_frame(self, message):
        data = message.encode('utf-8')
        length = len(data)
        frame = bytearray([0x81])

        if length <= 125:
            frame.append(length)
        elif length <= 65535:
            frame.append(126)
            frame.extend(struct.pack("!H", length))
        else:
            frame.append(127)
            frame.extend(struct.pack("!Q", length))

        frame.extend(data)
        return bytes(frame)
