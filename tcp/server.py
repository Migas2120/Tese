"""
tcp_server.py

Implements a non-blocking TCP server that accepts connections from Unity,
receives JSON messages, and routes them into the ROS system via `MiddleManNode`.

Features:
- Select-based socket multiplexing
- Handles multiple Unity connections
- Forwards decoded payloads to ROS via `publish_from_unity()`
- Logs all connections and exceptions
"""

import socket
import select
import logging

class TCPServer:
    """
    TCP server that receives inbound JSON commands from Unity and forwards them
    to a ROS 2 node for handling.
    """

    def __init__(self, ros_node, logger=None, host='127.0.0.1', port=65432):
        """
        Parameters:
        - ros_node: instance of MiddleManNode (used to forward received data)
        - logger: optional logger instance
        - host: server bind IP (defaults to localhost)
        - port: TCP port to listen on
        """
        self.ros_node = ros_node
        self.logger = logger or logging.getLogger(__name__)
        self.host = host
        self.port = port

        self.server_socket = None
        self.sockets_list = []
        self.clients = {}
        self.running = False

    def start(self):
        """Starts the TCP server and begins the socket listening loop."""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            #self.server_socket.bind(("172.18.12.6", self.port))
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen()
            self.server_socket.setblocking(False)

            self.sockets_list = [self.server_socket]
            self.running = True

            self.logger.info(f'MiddleMan TCP server listening on {self.host}:{self.port}')

            while self.running:
                # Multiplex across all sockets
                read_sockets, _, exception_sockets = select.select(
                    self.sockets_list, [], self.sockets_list, 0.1
                )

                for notified_socket in read_sockets:
                    if notified_socket == self.server_socket:
                        self._accept_connection()
                    else:
                        self._receive_message(notified_socket)

                for sock in exception_sockets:
                    self._handle_exception(sock)

        except Exception as e:
            self.logger.exception(f"Server encountered an error: {e}")
        finally:
            self.stop()

    def _accept_connection(self):
        """Accepts a new incoming client connection."""
        client_socket, client_address = self.server_socket.accept()
        client_socket.setblocking(False)
        self.sockets_list.append(client_socket)
        self.clients[client_socket] = client_address
        self.logger.info(f'New connection from {client_address}')

    def _receive_message(self, sock):
        """Receives a message from a client socket and forwards it to the ROS node."""
        try:
            message = sock.recv(1024)
            if not message:
                self._disconnect_client(sock)
                return

            decoded = message.decode()
            self.logger.debug(f'Received from {self.clients[sock]}: {decoded}')
            self.ros_node.publish_from_unity(decoded)

        except Exception as e:
            self.logger.error(f'Error with {self.clients.get(sock, "Unknown client")}: {e}')
            self._disconnect_client(sock)

    def _disconnect_client(self, sock):
        """Cleans up after a client disconnects or errors out."""
        addr = self.clients.get(sock, "Unknown client")
        self.logger.info(f'Connection closed from {addr}')
        self.sockets_list.remove(sock)
        self.clients.pop(sock, None)
        sock.close()

    def _handle_exception(self, sock):
        """Handles sockets that threw exceptions during select()."""
        self.logger.warning(f'Socket exception with {self.clients.get(sock, "Unknown client")}')
        self._disconnect_client(sock)

    def stop(self):
        """Stops the TCP server and closes all open sockets."""
        self.running = False
        for sock in self.sockets_list:
            sock.close()
        self.logger.info("TCP server stopped.")
