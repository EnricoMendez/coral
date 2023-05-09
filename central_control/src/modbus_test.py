from pymodbus.client.sync import ModbusTcpClient

# Define IP address and port number in UR5 and creat client
IP_ADDRESS = '192.168.0.102'
PORT = 502
client = ModbusTcpClient(IP_ADDRESS, PORT)
client.connect() # Connect with UR5

# Define digital inputs
input_value = False
address = 24

# write values to input

result = client.write_coil(address, input_value)  # send values
    # Verify send
if result.isError():
    print("Error al escribir en la entrada digital ", address)
else:
    print("Entrada digital ", address, " actualizada con éxito")
# Desconnect from robot
client.close()
