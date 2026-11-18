#!/usr/bin/env python3
# Copyright (c) 2026 FlyByWire Simulations
#
# SPDX-License-Identifier: GPL-3.0

"""Runs a JavaScript file in an MSFS instrument through the Coherent GT debugger and prints its console output.

The debugger (developer mode, port 19999) speaks the WebKit inspector protocol on ws://<host>:19999/devtools/page/<id>.
Standard library only. Stops when a console line contains --until, or after --timeout seconds.
Usage: coherent-eval.py --host <windows host ip> --page-title A32NX_MCDU script.js [--until text] [--result expr]
"""

import argparse
import base64
import json
import os
import socket
import struct
import sys
import time
import urllib.request


class WebSocket:
    def __init__(self, host, port, path):
        self.sock = socket.create_connection((host, port), timeout=10)
        key = base64.b64encode(os.urandom(16)).decode()
        self.sock.sendall(
            (
                f"GET {path} HTTP/1.1\r\nHost: {host}:{port}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n"
            ).encode()
        )
        header = b""
        while b"\r\n\r\n" not in header:
            chunk = self.sock.recv(1)
            if not chunk:
                raise ConnectionError("handshake closed")
            header += chunk
        if b" 101 " not in header.split(b"\r\n")[0]:
            raise ConnectionError(header.decode(errors="replace"))

    def _recv_exact(self, n):
        data = b""
        while len(data) < n:
            chunk = self.sock.recv(n - len(data))
            if not chunk:
                raise ConnectionError("closed")
            data += chunk
        return data

    def send(self, text):
        payload = text.encode()
        header = bytearray([0x81])
        n = len(payload)
        if n < 126:
            header.append(0x80 | n)
        elif n < 65536:
            header.append(0x80 | 126)
            header += struct.pack(">H", n)
        else:
            header.append(0x80 | 127)
            header += struct.pack(">Q", n)
        mask = os.urandom(4)
        header += mask
        self.sock.sendall(bytes(header) + bytes(b ^ mask[i % 4] for i, b in enumerate(payload)))

    def recv(self, timeout):
        self.sock.settimeout(timeout)
        message = b""
        while True:
            b0, b1 = self._recv_exact(2)
            n = b1 & 0x7F
            if n == 126:
                n = struct.unpack(">H", self._recv_exact(2))[0]
            elif n == 127:
                n = struct.unpack(">Q", self._recv_exact(8))[0]
            mask = self._recv_exact(4) if b1 & 0x80 else None
            data = self._recv_exact(n)
            if mask:
                data = bytes(b ^ mask[i % 4] for i, b in enumerate(data))
            opcode = b0 & 0x0F
            if opcode == 0x8:
                raise ConnectionError("closed by server")
            if opcode == 0x9:  # ping
                continue
            message += data
            if b0 & 0x80:
                return message.decode(errors="replace")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("script")
    ap.add_argument("--host", required=True)
    ap.add_argument("--port", type=int, default=19999)
    ap.add_argument("--page-title", required=True, help="substring of the page title, e.g. A32NX_MCDU")
    ap.add_argument("--until", help="stop when a console line contains this")
    ap.add_argument("--result", help="expression to evaluate at the end (JSON.stringify'd), printed last")
    ap.add_argument("--timeout", type=float, default=600)
    args = ap.parse_args()

    pages = json.load(urllib.request.urlopen(f"http://{args.host}:{args.port}/pagelist.json", timeout=10))
    matches = [p for p in pages if args.page_title in p.get("title", "")]
    if len(matches) != 1:
        sys.exit(f"{len(matches)} pages match {args.page_title!r}")
    page = matches[0]
    print(f"# page {page['id']}: {page['title']}", file=sys.stderr)

    ws = WebSocket(args.host, args.port, f"/devtools/page/{page['id']}")
    next_id = [0]

    def call(method, params=None):
        next_id[0] += 1
        ws.send(json.dumps({"id": next_id[0], "method": method, "params": params or {}}))
        return next_id[0]

    def wait_for(msg_id):
        deadline = time.time() + 60
        while time.time() < deadline:
            msg = json.loads(ws.recv(60))
            if msg.get("id") == msg_id:
                return msg
            print_console(msg)
        sys.exit("no reply")

    done = [False]

    def print_console(msg):
        if msg.get("method") == "Console.messageAdded":
            text = msg["params"]["message"].get("text", "")
            print(text, flush=True)
            if args.until and args.until in text:
                done[0] = True

    wait_for(call("Console.enable"))
    with open(args.script, encoding="utf-8") as f:
        reply = wait_for(call("Runtime.evaluate", {"expression": f.read(), "objectGroup": "coherent-eval"}))
    if reply.get("error") or reply.get("result", {}).get("wasThrown"):
        sys.exit(f"evaluate failed: {json.dumps(reply)[:2000]}")

    start = time.time()
    while not done[0] and time.time() - start < args.timeout:
        try:
            print_console(json.loads(ws.recv(5)))
        except socket.timeout:
            pass
    if not done[0]:
        print("# timeout", file=sys.stderr)

    if args.result:
        reply = wait_for(
            call("Runtime.evaluate", {"expression": f"JSON.stringify({args.result})", "returnByValue": True})
        )
        print(reply.get("result", {}).get("result", {}).get("value"))


if __name__ == "__main__":
    main()
