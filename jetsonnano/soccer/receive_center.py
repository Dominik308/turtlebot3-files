import socket

def main():
    # Create TCP socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        # Bind to all interfaces on port 9999
        server_socket.bind(('', 12345))
        server_socket.listen(1)
        print("Waiting for connection on port 12345...")
        
        while True:
            client_socket, addr = server_socket.accept()
            print(f"Connected to {addr}")
            
            try:
                buffer = b''
                while True:
                    data = client_socket.recv(1024)
                    if not data:
                        break
                    
                    buffer += data
                    while b'\n' in buffer:
                        line, buffer = buffer.split(b'\n', 1)
                        location = line.decode().strip()
                        
                        if location == "None":
                            print("No ball detected")
                        else:
                            try:
                                print(f"Received location: {location}")
                            except ValueError:
                                print(f"Invalid data received: {location}")
                                
            except ConnectionResetError:
                print("Client disconnected unexpectedly")
            finally:
                client_socket.close()
                print("Connection closed")
                
    except KeyboardInterrupt:
        print("\nServer shutdown requested")
    finally:
        server_socket.close()
        print("Server closed")

if __name__ == "__main__":
    main()
