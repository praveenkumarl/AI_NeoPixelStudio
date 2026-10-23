#!/usr/bin/env python3
r"""Decode ui_embedded.h back into the original web/index.html source.

The header is an auto-generated PROGMEM C string; this reverses it so the UI
can be edited as normal HTML and re-embedded with tools/build_embedded_ui.py.

C unescapes each string literal BEFORE concatenating them, so this decodes
fragment-by-fragment - joining first would merge a trailing escape of one
fragment into the hex digits that start the next one.
"""
import re
import sys

SRC = sys.argv[1] if len(sys.argv) > 1 else "ui_embedded.h"
DST = sys.argv[2] if len(sys.argv) > 2 else "web/index.html"

with open(SRC, "r", encoding="utf-8") as f:
    text = f.read()

# Collect every quoted fragment of the UI_EMBEDDED[] literal, in order.
# Only look after the PROGMEM declaration so comment text with quotes is
# ignored; stop at the last terminator before #endif because the blob itself
# contains raw quote-semicolon sequences that would fool a first-match search.
start = text.index("UI_EMBEDDED[] PROGMEM =")
# The last content line ends with '";' (the declaration terminator), so the
# slice must include that closing quote or the final fragment is dropped.
end = text.rindex('";', 0, text.index("#endif")) + 1
frags = re.findall(r'"((?:[^"\\]|\\.)*)"', text[start:end], re.S)

SIMPLE = {"n": b"\n", "t": b"\t", "r": b"\r", '"': b'"', "'": b"'",
          "\\": b"\\", "?": b"?", "0": b"\x00", "a": b"\a", "b": b"\b",
          "f": b"\f", "v": b"\v"}

out = bytearray()
for frag in frags:
    i = 0
    while i < len(frag):
        ch = frag[i]
        if ch != "\\":
            out += ch.encode("utf-8")
            i += 1
            continue
        nxt = frag[i + 1]
        if nxt in SIMPLE:
            out += SIMPLE[nxt]
            i += 2
        elif nxt in "xX":
            m = re.match(r"[xX]([0-9a-fA-F]{1,2})", frag[i + 1:])
            out.append(int(m.group(1), 16))
            i += 1 + len(m.group(0))
        else:
            raise SystemExit(f"unknown escape {frag[i:i+4]!r} in fragment")

with open(DST, "wb") as f:
    f.write(bytes(out))

print(f"decoded {len(frags)} fragments, {len(out)} bytes -> {DST}")
