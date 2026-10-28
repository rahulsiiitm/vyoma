#!/usr/bin/env python3
"""Forward a container's X11 connection to the local WSLg desktop socket."""

import argparse
import select
import socket
import socketserver


class X11Connection(socketserver.BaseRequestHandler):
    def handle(self):
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as desktop:
            desktop.connect('/tmp/.X11-unix/X0')
            connections = (self.request, desktop)
            while True:
                ready, _, _ = select.select(connections, [], [])
                for source in ready:
                    data = source.recv(65536)
                    if not data:
                        return
                    destination = desktop if source is self.request else self.request
                    destination.sendall(data)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--listen', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=6000)
    args = parser.parse_args()
    class Server(socketserver.ThreadingTCPServer):
        allow_reuse_address = True
        daemon_threads = True
    with Server((args.listen, args.port), X11Connection) as server:
        server.serve_forever()


if __name__ == '__main__':
    main()
