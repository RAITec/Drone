import serial
import socket

print("Abrindo portas!")


#ser = serial.Serial('/dev/ttyUSB2', 115200)
port1 = serial.Serial('/dev/tnt0', 115200)
port2 = serial.Serial('/dev/tnt1', 115200)

''''
if (port1.is_open and port2.is_open):
    print('Portas abertas com succeso!')


while ser.is_open:
    try:
        line = ser.readline()
        port1.write(line)
        port2.write(line)
        line = line.decode('utf-8')
        print(line)
    except:
        pass
'''



host = "192.168.67.61"  # Aceita conexões de qualquer IP
port = 5000      # Porta para ouvir o ESP32


# Configura o servidor

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server_socket:
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    #server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    #server_socket.settimeout(5)
    
    try:
        server_socket.bind((host, port))
        print(f'Servido ouvindo no ip {host}, porta {port}')
    except Exception as e:
        print(f"Erro ao vincular o socket à porta: {e}")
        exit(1)

    print("Aguardando conexão do ESP32...")

    # Loop para aceitar conexões    
    while True:      
        try:
            data, addr = server_socket.recvfrom(1024)
            message = data.decode("utf-8")
            port1.write(data)
            port2.write(data)
            print(f"{message}")
        except socket.timeout:
            print("Nenhuma mensagem recebida (timeout). Continuando...")
        except Exception as e:
            print(f"Erro ao processar pacotes: {e}")
            
         
port1.close()
port2.close()
        
