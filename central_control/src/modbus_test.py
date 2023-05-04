from pymodbus.client.sync import ModbusTcpClient

# Definir la dirección IP y el número de puerto del robot UR5
IP_ADDRESS = '192.168.1.10'
PORT = 502

# Crear un cliente Modbus TCP
client = ModbusTcpClient(IP_ADDRESS, PORT)

# Conectar con el robot UR5
client.connect()

# Definir los valores de entrada digital
input_values = [True, False, True, False, True, False]

# Escribir los valores en las entradas digitales del robot UR5
for i in range(len(input_values)):
    # Escribir en la entrada digital correspondiente
    result = client.write_coil(i, input_values[i])
    
    # Verificar si se escribió correctamente
    if result.isError():
        print("Error al escribir en la entrada digital ", i)
    else:
        print("Entrada digital ", i, " actualizada con éxito")

# Desconectar del robot UR5
client.close()
