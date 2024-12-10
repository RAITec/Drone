import csv
import serial  # Importa a biblioteca pyserial

# Conecta à porta serial
ser = serial.Serial('/dev/ttyUSB0', 115200)  

# Abre o arquivo CSV para escrita
with open('dados_arduino.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    # Escreve o cabeçalho do arquivo CSV
    writer.writerow(['Motor1', 'Motor2', 'Motor3', 'Motor4', 'Angle_Roll', 'Angle_Pitch', 'Kalman_Angle_Roll', 'Kalman_Angle_Pitch'])

    try:
        while True:
            # Lê uma linha de dados da serial
            line = ser.readline().decode('utf-8').strip()
            print(line)  # Exibe o dado recebido no console
            
            # Verifica se a linha recebida contém os valores dos motores
            if "Motor1:" in line:
                # Divide a linha recebida por vírgula para separar os valores
                values = line.split(',')
                
                # Extrai os valores de cada motor
                motor1 = values[0].split(':')[1]
                motor2 = values[1].split(':')[1]
                motor3 = values[2].split(':')[1]
                motor4 = values[3].split(':')[1]

                angle_roll = values[4].split(':')[1]
                print(angle_roll)

                angle_pitch = values[5].split(':')[1]
                kalman_angle_roll = values[6].split(':')[1]
                kalman_angle_pitch = values[7].split(':')[1]
        
                writer.writerow([motor1, motor2, motor3, motor4, angle_roll, angle_pitch, kalman_angle_roll, kalman_angle_pitch])

    except KeyboardInterrupt:
        print("Interrupção pelo usuário")
    finally:
        ser.close()